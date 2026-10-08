#ifndef GAME_CLIENT_COMPONENTS_FREEZE_ASSIST_H
#define GAME_CLIENT_COMPONENTS_FREEZE_ASSIST_H

#include <base/vmath.h>

#include <engine/client/enums.h>

#include <game/client/component.h>
#include <game/gamecore.h>

#include <generated/protocol.h>

#include <cstdint>
#include <vector>

/**
 * Freeze avoidance assist.
 *
 * A receding-horizon (model predictive) input filter. Once per input tick it simulates the local tee forward with the
 * real CCharacterCore physics, once with the player's raw input held ("baseline") and once for a small set of
 * alternative input plans. If the baseline touches a freeze-family tile inside the look-ahead window, the cheapest
 * plan that avoids all hazards replaces the first-tick input that is about to be sent.
 *
 * DESYNC SAFETY
 * The filter only ever runs from CControls::SnapInput(), i.e. at the single point where a tick's input is created.
 * The filtered input is what gets stored in CClient::m_aInputs[], sent to the server, and replayed by the prediction
 * (CGameClient::OnPredict -> Client()->GetInput(Tick)). Server and client therefore simulate the identical input and
 * nothing in here touches predicted or snapshot state: all simulation happens on private copies of the cores.
 */
class CFreezeAssist : public CComponent
{
public:
	int Sizeof() const override { return sizeof(*this); }
	void OnMapLoad() override;
	void OnReset() override;

	/**
	 * Filters the raw input of the controlled tee in place.
	 *
	 * @param Dummy Index of the controlled connection (g_Config.m_ClDummy).
	 * @param pInput Input that is about to be sent. Only direction, jump, hook and (for hook plans) the target are changed.
	 * @return Whether the input was modified.
	 */
	bool FilterInput(int Dummy, CNetObj_PlayerInput *pInput);

	/**
	 * Whether the filtered input differs from the last one that was actually sent. CControls only re-sends when the raw
	 * input changed, so an override that appears or disappears on its own has to force a send.
	 */
	bool DiffersFromLastSent(int Dummy, const CNetObj_PlayerInput &Input) const;
	void NoteSent(int Dummy, const CNetObj_PlayerInput &Input);

	bool Engaged() const { return m_Engaged; }

private:
	static constexpr int MAX_NEARBY = 8;
	static constexpr int NUM_HOOK_AIMS = 8;
	static constexpr int JUMP_FOLLOW = -2; // keep the player's jump key as is
	static constexpr int JUMP_NEVER = -1; // key released for the whole window
	// >= 0: jump key is pressed for exactly one tick, that many ticks from now
	static constexpr int HOOK_FOLLOW = -2;
	static constexpr int HOOK_RELEASE = -1;
	// >= 0: hook held towards aim direction number n (n * 45 degrees)

	struct SPlan
	{
		int m_Direction = 0;
		int m_JumpTick = JUMP_FOLLOW;
		int m_HookAim = HOOK_FOLLOW;
		bool SameSteering(const SPlan &Other) const { return m_Direction == Other.m_Direction && m_HookAim == Other.m_HookAim; }
	};

	struct SOutcome
	{
		bool m_Hit = false;
		int m_HitTick = 0;
		float m_Violation = 0.0f; // accumulated clearance-margin violation in "ticks"
		bool m_SpentJump = false;
		vec2 m_EndPos = vec2(0.0f, 0.0f);
		float m_Cost = 0.0f;
	};

	/**
	 * Private physics sandbox. Nothing in here aliases state of the real prediction: cores are copies, the world is
	 * a separate CWorldCore and the anti-ping callbacks are replaced by no-ops.
	 */
	struct SScratch
	{
		CWorldCore m_World;
		CCharacterCore m_Self;
		CCharacterCore m_aNear[MAX_NEARBY];
		int m_aNearId[MAX_NEARBY] = {};
		int m_NumNear = 0;
		CCharacterCore m_Sim;
		CCharacterCore m_aSimNear[MAX_NEARBY];
		int m_SelfId = -1;
		int m_Team = 0;
	};

	bool CanRun(int Dummy, int *pLocalId) const;
	void BuildScratch(int LocalId);
	SOutcome Simulate(const SPlan &Plan, const CNetObj_PlayerInput &Raw, int Horizon, bool StopOnHit);
	float PlanCost(const SPlan &Plan, const SOutcome &Out, const SOutcome &Baseline, const CNetObj_PlayerInput &Raw) const;
	static CNetObj_PlayerInput InputFor(const SPlan &Plan, const CNetObj_PlayerInput &Raw, int Tick);
	static vec2 AimVector(int Aim);

	bool IsHazardIndex(int Index) const;
	bool SweepHazard(vec2 From, vec2 To) const;
	float ClearancePx(vec2 Pos) const;

	// static clearance field (3-4 chamfer distance to the nearest freeze tile, in 1/3 tile units)
	std::vector<uint8_t> m_vField;
	int m_FieldWidth = 0;
	int m_FieldHeight = 0;

	SScratch m_Scratch;

	// temporal coherence between consecutive ticks
	bool m_Engaged = false;
	SPlan m_PrevPlan;
	int m_PrevPlanTick = -1;

	// FilterInput() may be called several times for the same tick (once per snapshot and once per tick)
	int m_CacheTick = -1;
	int m_CacheDummy = -1;
	CNetObj_PlayerInput m_CacheRaw;
	CNetObj_PlayerInput m_CacheOut;
	bool m_CacheOverridden = false;

	CNetObj_PlayerInput m_aLastSent[NUM_DUMMIES];
};

#endif
