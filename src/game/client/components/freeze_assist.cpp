#include "freeze_assist.h"

#include <base/log.h>
#include <base/math.h>
#include <base/net.h>
#include <base/mem.h>

#include <engine/client.h>
#include <engine/shared/config.h>

#include <game/client/gameclient.h>
#include <game/collision.h>
#include <game/mapitems.h>
#include <game/teamscore.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr float TILE = 32.0f;
constexpr float NEARBY_RADIUS = 420.0f; // slightly more than the hook length
constexpr int MIN_HORIZON = 8;
constexpr int MAX_HORIZON = 60;
constexpr int FIELD_MAX = 60; // 20 tiles in 1/3 tile chamfer units

// Cost model. Units are loosely "how much does this hurt the player's intent".
constexpr float COST_HIT = 1000.0f;
constexpr float COST_DIRECTION = 3.0f;
constexpr float COST_JUMP = 4.0f;
constexpr float COST_AIR_JUMP = 6.0f; // the double jump is a resource, don't burn it unless it saves the tee
constexpr float COST_HOOK = 8.0f;
constexpr float COST_CLEARANCE = 2.0f;
constexpr float COST_DISPLACEMENT = 0.5f; // per tile of end position drift vs. the player's own trajectory
constexpr float COST_SWITCH = 2.0f; // hysteresis: don't flip-flop between steering plans on consecutive ticks

bool IsFreezeFamily(int Tile)
{
	return Tile == TILE_FREEZE || Tile == TILE_DFREEZE || Tile == TILE_LFREEZE;
}

bool SameInputKeys(const CNetObj_PlayerInput &A, const CNetObj_PlayerInput &B)
{
	return A.m_Direction == B.m_Direction && A.m_Jump == B.m_Jump && A.m_Hook == B.m_Hook;
}

// The aim is only taken over when a hook plan deliberately steers it; otherwise the player's current aim stays.
void CopySteering(CNetObj_PlayerInput *pTo, const CNetObj_PlayerInput &From, bool CopyAim)
{
	pTo->m_Direction = From.m_Direction;
	pTo->m_Jump = From.m_Jump;
	pTo->m_Hook = From.m_Hook;
	if(CopyAim)
	{
		pTo->m_TargetX = From.m_TargetX;
		pTo->m_TargetY = From.m_TargetY;
	}
}
} // namespace

// ---------------------------------------------------------------------------------------------------------------------
// Map data
// ---------------------------------------------------------------------------------------------------------------------

void CFreezeAssist::OnMapLoad()
{
	m_vField.clear();
	m_FieldWidth = Collision()->GetWidth();
	m_FieldHeight = Collision()->GetHeight();
	if(m_FieldWidth <= 0 || m_FieldHeight <= 0)
		return;

	const int W = m_FieldWidth;
	const int H = m_FieldHeight;
	m_vField.assign((size_t)W * H, (uint8_t)FIELD_MAX);

	const CCollision *pCol = Collision();
	for(int i = 0; i < W * H; i++)
	{
		if(IsFreezeFamily(pCol->GetTileIndex(i)) || IsFreezeFamily(pCol->GetFrontTileIndex(i)))
			m_vField[i] = 0;
	}

	// two-pass 3-4 chamfer distance transform: orthogonal step = 3, diagonal step = 4 (one tile ~ 3 units)
	auto Relax = [&](int Index, int NeighbourIndex, int Cost) {
		const int Candidate = std::min<int>(m_vField[NeighbourIndex] + Cost, FIELD_MAX);
		if(Candidate < m_vField[Index])
			m_vField[Index] = (uint8_t)Candidate;
	};
	for(int y = 0; y < H; y++)
	{
		for(int x = 0; x < W; x++)
		{
			const int i = y * W + x;
			if(x > 0)
				Relax(i, i - 1, 3);
			if(y > 0)
			{
				Relax(i, i - W, 3);
				if(x > 0)
					Relax(i, i - W - 1, 4);
				if(x < W - 1)
					Relax(i, i - W + 1, 4);
			}
		}
	}
	for(int y = H - 1; y >= 0; y--)
	{
		for(int x = W - 1; x >= 0; x--)
		{
			const int i = y * W + x;
			if(x < W - 1)
				Relax(i, i + 1, 3);
			if(y < H - 1)
			{
				Relax(i, i + W, 3);
				if(x < W - 1)
					Relax(i, i + W + 1, 4);
				if(x > 0)
					Relax(i, i + W - 1, 4);
			}
		}
	}
}

void CFreezeAssist::OnReset()
{
	m_Engaged = false;
	m_PrevPlanTick = -1;
	m_CacheTick = -1;
	m_CacheOverridden = false;
	mem_zero(m_aLastSent, sizeof(m_aLastSent));
}

float CFreezeAssist::ClearancePx(vec2 Pos) const
{
	const int X = std::clamp((int)std::floor(Pos.x / TILE), 0, m_FieldWidth - 1);
	const int Y = std::clamp((int)std::floor(Pos.y / TILE), 0, m_FieldHeight - 1);
	// field value is centre-to-centre; subtract half a tile to approximate the distance to the hazard's edge
	return std::max(0.0f, m_vField[(size_t)Y * m_FieldWidth + X] * (TILE / 3.0f) - TILE / 2.0f);
}

// ---------------------------------------------------------------------------------------------------------------------
// Hazard queries (mirror CCharacter::HandleTiles / DDRaceTick: freeze triggers on the tile of the tee's centre)
// ---------------------------------------------------------------------------------------------------------------------

bool CFreezeAssist::IsHazardIndex(int Index) const
{
	const CCollision *pCol = Collision();
	const int Tile = pCol->GetTileIndex(Index);
	const int Front = pCol->GetFrontTileIndex(Index);
	if(IsFreezeFamily(Tile) || IsFreezeFamily(Front))
		return true;
	if(g_Config.m_ClFreezeAssistDeath && (Tile == TILE_DEATH || Front == TILE_DEATH))
		return true;

	// switch-layer freeze is only dangerous while its switch is active for our team
	if(pCol->GetSwitchType(Index) == TILE_FREEZE)
	{
		const int Number = pCol->GetSwitchNumber(Index);
		const auto &vSwitchers = m_Scratch.m_World.m_vSwitchers;
		return Number == 0 || (Number > 0 && Number < (int)vSwitchers.size() && m_Scratch.m_Team >= 0 && m_Scratch.m_Team < NUM_DDRACE_TEAMS && vSwitchers[Number].m_aStatus[m_Scratch.m_Team]);
	}
	return false;
}

// Exact grid traversal (Amanatides & Woo): visits every tile the segment passes through, so fast tees can't tunnel.
bool CFreezeAssist::SweepHazard(vec2 From, vec2 To) const
{
	constexpr float Inf = std::numeric_limits<float>::infinity();
	const int W = m_FieldWidth;
	const int H = m_FieldHeight;

	int X = (int)std::floor(From.x / TILE);
	int Y = (int)std::floor(From.y / TILE);
	const int EndX = (int)std::floor(To.x / TILE);
	const int EndY = (int)std::floor(To.y / TILE);
	const vec2 D = To - From;
	const int StepX = D.x > 0.0f ? 1 : -1;
	const int StepY = D.y > 0.0f ? 1 : -1;
	const float InvX = D.x != 0.0f ? 1.0f / std::fabs(D.x) : Inf;
	const float InvY = D.y != 0.0f ? 1.0f / std::fabs(D.y) : Inf;
	float MaxX = D.x != 0.0f ? (StepX > 0 ? (X + 1) * TILE - From.x : From.x - X * TILE) * InvX : Inf;
	float MaxY = D.y != 0.0f ? (StepY > 0 ? (Y + 1) * TILE - From.y : From.y - Y * TILE) * InvY : Inf;
	const float DeltaX = TILE * InvX;
	const float DeltaY = TILE * InvY;

	int Guard = std::abs(EndX - X) + std::abs(EndY - Y) + 2;
	while(true)
	{
		const int Cx = std::clamp(X, 0, W - 1);
		const int Cy = std::clamp(Y, 0, H - 1);
		if(IsHazardIndex(Cy * W + Cx))
			return true;
		if((X == EndX && Y == EndY) || --Guard <= 0)
			return false;
		if(MaxX < MaxY)
		{
			X += StepX;
			MaxX += DeltaX;
		}
		else
		{
			Y += StepY;
			MaxY += DeltaY;
		}
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Sandbox
// ---------------------------------------------------------------------------------------------------------------------

bool CFreezeAssist::CanRun(int Dummy, int *pLocalId) const
{
	if(!g_Config.m_ClFreezeAssist || Client()->State() != IClient::STATE_ONLINE)
		return false;

	// Opt-in per session for public servers. Input automation is against the rules of most of them (DDNet ranks included).
	if(!g_Config.m_ClFreezeAssistOnline && !net_addr_is_local(&Client()->ServerAddress()))
		return false;

	if(!GameClient()->Predict())
		return false;

	const int LocalId = GameClient()->m_aLocalIds[Dummy];
	if(LocalId < 0 || LocalId >= MAX_CLIENTS)
		return false;

	const CGameClient::CClientData &Data = GameClient()->m_aClients[LocalId];
	if(!Data.m_Active || Data.m_Paused || Data.m_Spec)
		return false;

	// Frozen / immune tees can't be helped (or don't need help)
	const CCharacterCore &Core = Data.m_Predicted;
	if(Core.m_Super || Core.m_Invincible || Core.m_DeepFrozen || Core.m_IsInFreeze || Core.m_FreezeEnd != 0)
		return false;

	*pLocalId = LocalId;
	return true;
}

void CFreezeAssist::BuildScratch(int LocalId)
{
	CGameClient *pClient = GameClient();
	SScratch &S = m_Scratch;
	CGameWorld &Real = pClient->m_PredictedWorld;
	CCollision *pCollision = Real.Collision();
	CTeamsCore *pTeams = Real.Teams();

	S.m_World = CWorldCore(); // no characters, no PRNG (tele-outs resolve to the first one: deterministic)
	S.m_World.m_vSwitchers = Real.m_Core.m_vSwitchers;
	S.m_SelfId = LocalId;
	S.m_Team = pTeams->Team(LocalId);

	// The latest predicted core is the state *before* the input that is being created right now is applied.
	S.m_Self = pClient->m_aClients[LocalId].m_Predicted;
	S.m_Self.m_Id = LocalId;
	S.m_Self.SetCoreWorld(&S.m_World, pCollision, pTeams);
	S.m_Self.SetAntiPingInterfereCallback([](int, bool) {}); // the copied callback would poke the real prediction
	S.m_Sim = S.m_Self;
	S.m_World.m_apCharacters[LocalId] = &S.m_Sim;

	// Nearby tees take part as static obstacles / hook targets. Their pristine copies are restored before every rollout.
	S.m_NumNear = 0;
	float aNearDist[MAX_NEARBY] = {};
	for(int i = 0; i < MAX_CLIENTS; i++)
	{
		const CCharacterCore *pOther = Real.m_Core.m_apCharacters[i];
		if(i == LocalId || !pOther)
			continue;
		const float Dist = distance(pOther->m_Pos, S.m_Self.m_Pos);
		if(Dist > NEARBY_RADIUS)
			continue;

		int Slot = S.m_NumNear;
		if(Slot == MAX_NEARBY)
		{
			Slot = (int)(std::max_element(aNearDist, aNearDist + MAX_NEARBY) - aNearDist);
			if(aNearDist[Slot] <= Dist)
				continue;
		}
		else
		{
			S.m_NumNear++;
		}
		aNearDist[Slot] = Dist;
		S.m_aNearId[Slot] = i;
		S.m_aNear[Slot] = *pOther;
		S.m_aNear[Slot].m_Id = i;
		S.m_aNear[Slot].SetCoreWorld(&S.m_World, pCollision, pTeams);
		S.m_aNear[Slot].SetAntiPingInterfereCallback([](int, bool) {});
	}
	for(int k = 0; k < S.m_NumNear; k++)
	{
		S.m_aSimNear[k] = S.m_aNear[k];
		S.m_World.m_apCharacters[S.m_aNearId[k]] = &S.m_aSimNear[k];
	}
}

vec2 CFreezeAssist::AimVector(int Aim)
{
	const float Angle = (float)Aim * (pi / 4.0f);
	return vec2(std::cos(Angle), std::sin(Angle)) * 100.0f;
}

CNetObj_PlayerInput CFreezeAssist::InputFor(const SPlan &Plan, const CNetObj_PlayerInput &Raw, int Tick)
{
	CNetObj_PlayerInput In = Raw;
	In.m_Direction = Plan.m_Direction;
	if(Plan.m_JumpTick != JUMP_FOLLOW)
		In.m_Jump = Plan.m_JumpTick == Tick ? 1 : 0;
	if(Plan.m_HookAim == HOOK_RELEASE)
	{
		In.m_Hook = 0;
	}
	else if(Plan.m_HookAim >= 0)
	{
		const vec2 Aim = AimVector(Plan.m_HookAim);
		In.m_Hook = 1;
		In.m_TargetX = round_to_int(Aim.x);
		In.m_TargetY = round_to_int(Aim.y);
	}
	return In;
}

CFreezeAssist::SOutcome CFreezeAssist::Simulate(const SPlan &Plan, const CNetObj_PlayerInput &Raw, int Horizon, bool StopOnHit)
{
	SScratch &S = m_Scratch;
	S.m_Sim = S.m_Self;
	for(int k = 0; k < S.m_NumNear; k++)
		S.m_aSimNear[k] = S.m_aNear[k];

	const float MarginPx = (float)g_Config.m_ClFreezeAssistMargin;
	SOutcome Out;
	for(int t = 0; t < Horizon; t++)
	{
		S.m_Sim.m_Input = InputFor(Plan, Raw, t);
		const vec2 From = S.m_Sim.m_Pos;

		// same order as CCharacter::Tick() + CCharacter::TickDeferred()
		S.m_Sim.Tick(true, true);
		S.m_Sim.Move();
		S.m_Sim.Quantize();

		if(!Out.m_Hit && SweepHazard(From, S.m_Sim.m_Pos))
		{
			Out.m_Hit = true;
			Out.m_HitTick = t;
			if(StopOnHit)
				break;
		}
		if(MarginPx > 0.0f)
			Out.m_Violation += std::max(0.0f, MarginPx - ClearancePx(S.m_Sim.m_Pos)) / MarginPx;
	}
	Out.m_EndPos = S.m_Sim.m_Pos;
	Out.m_SpentJump = (S.m_Sim.m_Jumped & 2) && !(S.m_Self.m_Jumped & 2);
	return Out;
}

float CFreezeAssist::PlanCost(const SPlan &Plan, const SOutcome &Out, const SOutcome &Baseline, const CNetObj_PlayerInput &Raw) const
{
	float Cost = 0.0f;

	// how far does the plan deviate from what the player is doing?
	if(Plan.m_Direction != Raw.m_Direction)
		Cost += COST_DIRECTION * ((Raw.m_Direction != 0 && Plan.m_Direction != 0) ? 1.5f : 1.0f); // reversing is worse than stopping
	const bool JumpDeviates = Plan.m_JumpTick == JUMP_NEVER ? Raw.m_Jump != 0 : (Plan.m_JumpTick >= 0 && !(Plan.m_JumpTick == 0 && Raw.m_Jump));
	if(JumpDeviates)
		Cost += COST_JUMP;
	const bool HookDeviates = Plan.m_HookAim == HOOK_RELEASE ? Raw.m_Hook != 0 : Plan.m_HookAim >= 0;
	if(HookDeviates)
		Cost += COST_HOOK;

	// side effects
	if(Out.m_SpentJump && !Baseline.m_SpentJump)
		Cost += COST_AIR_JUMP;
	Cost += Out.m_Violation * COST_CLEARANCE;
	Cost += distance(Out.m_EndPos, Baseline.m_EndPos) / TILE * COST_DISPLACEMENT;

	// temporal coherence
	if(m_Engaged && !Plan.SameSteering(m_PrevPlan))
		Cost += COST_SWITCH;
	return Cost;
}

// ---------------------------------------------------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------------------------------------------------

bool CFreezeAssist::FilterInput(int Dummy, CNetObj_PlayerInput *pInput)
{
	const CNetObj_PlayerInput Raw = *pInput;

	int LocalId = -1;
	if(m_vField.empty() || !CanRun(Dummy, &LocalId))
	{
		m_Engaged = false;
		m_CacheTick = -1;
		return false;
	}

	// SendInput() can run more than once per prediction tick (snapshot arrival + tick advance). Keep the answer stable.
	const int Tick = Client()->PredGameTick(Dummy);
	if(m_CacheTick == Tick && m_CacheDummy == Dummy && SameInputKeys(Raw, m_CacheRaw))
	{
		if(m_CacheOverridden)
			CopySteering(pInput, m_CacheOut, m_PrevPlan.m_HookAim >= 0);
		return m_CacheOverridden;
	}

	const int Horizon = std::clamp((int)g_Config.m_ClFreezeAssistHorizon, MIN_HORIZON, MAX_HORIZON);
	BuildScratch(LocalId);

	const SPlan Baseline = {Raw.m_Direction, JUMP_FOLLOW, HOOK_FOLLOW};
	const SOutcome Base = Simulate(Baseline, Raw, Horizon, false);
	const bool Dangerous = Base.m_Hit || Base.m_Violation > 0.0f;

	bool Overridden = false;
	if(Dangerous)
	{
		const float BaseCost = Base.m_Hit ? COST_HIT : Base.m_Violation * COST_CLEARANCE;

		std::vector<int> vJumpModes = {JUMP_FOLLOW};
		if(g_Config.m_ClFreezeAssistJump)
			vJumpModes.insert(vJumpModes.end(), {JUMP_NEVER, 0, 1, 3});
		std::vector<int> vHookModes = {HOOK_FOLLOW};
		if(g_Config.m_ClFreezeAssistHook)
		{
			if(Raw.m_Hook)
				vHookModes.push_back(HOOK_RELEASE);
			for(int Aim = 0; Aim < NUM_HOOK_AIMS; Aim++)
				vHookModes.push_back(Aim);
		}

		bool Found = false;
		SPlan BestPlan;
		float BestCost = 0.0f;
		const int aDirections[3] = {Raw.m_Direction, Raw.m_Direction == 0 ? -1 : 0, Raw.m_Direction == 1 ? -1 : 1};
		for(const int Direction : aDirections)
		{
			for(const int Jump : vJumpModes)
			{
				if(Jump == JUMP_NEVER && !Raw.m_Jump)
					continue; // identical to following the key
				for(const int Hook : vHookModes)
				{
					if(Direction == Raw.m_Direction && Jump == JUMP_FOLLOW && Hook == HOOK_FOLLOW)
						continue; // that is the baseline
					if(Hook != HOOK_FOLLOW && Jump >= 1)
						continue; // keep the search space small: delayed jumps are not combined with hooks

					const SPlan Plan = {Direction, Jump, Hook};
					SOutcome Out = Simulate(Plan, Raw, Horizon, true);
					if(Out.m_Hit)
						continue;
					Out.m_Cost = PlanCost(Plan, Out, Base, Raw);
					if(!Found || Out.m_Cost < BestCost)
					{
						Found = true;
						BestPlan = Plan;
						BestCost = Out.m_Cost;
					}
				}
			}
		}

		// If no plan escapes (e.g. a one-tile freeze gap at full speed) the player's own input stays untouched.
		if(Found && BestCost < BaseCost)
		{
			CopySteering(pInput, InputFor(BestPlan, Raw, 0), BestPlan.m_HookAim >= 0);
			m_PrevPlan = BestPlan;
			m_PrevPlanTick = Tick;
			Overridden = true;

			if(g_Config.m_ClFreezeAssistDebug)
			{
				log_info("freeze_assist", "tick=%d hit_in=%d dir %d->%d jump=%d hook=%d cost=%.1f", Tick, Base.m_Hit ? Base.m_HitTick : -1, Raw.m_Direction, BestPlan.m_Direction, BestPlan.m_JumpTick, BestPlan.m_HookAim, BestCost);
			}
		}
		else if(g_Config.m_ClFreezeAssistDebug && Base.m_Hit)
		{
			log_info("freeze_assist", "tick=%d unavoidable hit in %d ticks, not intervening", Tick, Base.m_HitTick);
		}
	}

	m_Engaged = Overridden;
	m_CacheTick = Tick;
	m_CacheDummy = Dummy;
	m_CacheRaw = Raw;
	m_CacheOut = *pInput;
	m_CacheOverridden = Overridden;
	return Overridden;
}

bool CFreezeAssist::DiffersFromLastSent(int Dummy, const CNetObj_PlayerInput &Input) const
{
	return !SameInputKeys(Input, m_aLastSent[Dummy]);
}

void CFreezeAssist::NoteSent(int Dummy, const CNetObj_PlayerInput &Input)
{
	m_aLastSent[Dummy] = Input;
}
