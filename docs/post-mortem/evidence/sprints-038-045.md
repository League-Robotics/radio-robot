# Evidence extract: Sprints 038–045 (FRC Elite Architecture migration arc, Phase 0→F)

(Compiled by a reader agent. Quotes verbatim from artifacts.)

## Arc-level context
The entire batch executes a single planning artifact: `042-.../issues/done/migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md`. Motivation: "Re-organize the firmware to adopt the *FRC Elite Architecture*... its three structural seams, its vendor/transport confinement rules, and its sim/test discipline."

The issue frames this as **rework of structure built in sprints 1–37**: "**The good news.** The codebase is already ~80% this shape... **This is a rename-by-capability + leak-sealing + seam-naming + sim-untangling job — not a rewrite.**"

Structures replaced map to earlier sprints: hal/Hardware factory (020), MockHAL/MockMotor noise model (021), EKF fusion (022–023), MotionController (017–018), the one-tree test layout (037 — re-tiered again ONE sprint later in 038). Stakeholder opportunity cost recorded: "**Decisions locked (this session):** full reorganization (all seams)... start now, **pausing the encoder-calibration mission**."

Migration fixed real latent defects in the old sim: "`sim_set_enc_l/r` lies (sets commanded rather than true travel); `BenchOtos` integrates commanded velocity while `ExactPose` uses true velocity... canonical midpoint-arc integration is triplicated" (040 sprint.md).

## Per-sprint highlights
- **038 (Phase 0)**: test tiers + three canaries (vendor-confinement grep gate, config field-pin, golden-TLM byte-exact). Re-restructures the test tree consolidated one sprint earlier (037). "The canaries are the migration's regression harness."
- **039 (Phase A)**: capability-typed device layer; renames sprint-020 HAL. Alias shims deliberately created for deletion in Phase F — planned double-touching.
- **040 (Phase B)**: PhysicsWorld replaces welded mock-sim from 021; fixed lying `sim_set_enc_l/r`. Pre-authorized escalation ladder in architecture-update (never needed). "Preserve the encoder sub-step expression verbatim... No algebraic simplification" — ULP-exact preservation to keep the golden-TLM canary meaningful.
- **041 (Phase C)**: PhysicalStateEstimate seam wraps EKF (022–023). Transition mirror kept for byte-identity, removed in F. The EKF file move quietly broke `coverage.sh` — not noticed until 045.
- **042 (Phase D)**: thin Superstructure; centralizes keepalive/SAFE/ESTOP scattered across loopTickOnce (from 017–018/024/026). Explicit anti-gold-plating guard stated three times.
- **043 (Phase E)**: subsystem wrapping, bodies verbatim.
- **044 (Phase F)**: scaffold demolition; "After this ticket the migration is complete. The codebase fully embodies the FRC Elite Architecture." REPLAY mode exited compiled-but-never-run.
- **045**: coverage to 86.2% simulatable; fixed the harness broken since Phase C. Coverage push surfaced latent defects carried verbatim through all six phases:
  - "SENSOR stop... on the QUEUE path (the sim's only mode) the stop is silently dropped" — real firmware bug found, pinned not fixed (test_sensor_stop_dropped_on_queue_path_documented).
  - "`startDriveClean` vs `startDrive`: NO live callers"; single-wheel ZOH branches "DEAD-IN-SIM".
  - "`RatioPidController.cpp` | **Dead code**... removed from live control loop by N13/030-010" — dead code migrated faithfully through all six phases.

## Batch synthesis (verbatim from reader)
Sprints 38–45 were an almost purely structural era: one master issue, seven phases, 35 tickets, essentially zero new robot behavior — contract was "structural changes only — no behavior changes... Move behavioral bodies verbatim," gated by three canaries. As execution, the phased approach demonstrably worked: every sprint's tickets/done matches its plan exactly, no reopened tickets or exceptions, pre-authorized escalation ladders never invoked, canaries held byte-exact through six phases — a strikingly clean run compared to debugging-heavy eras elsewhere. The costs were quieter: the issue's hard constraint of a per-phase "hardware bench smoke" vanishes from every sprint DoD after 038 (validation was sim-only); the coverage harness silently broke in Phase C and stayed broken until 045; REPLAY closed as an unexercised stub; verbatim-move discipline preserved dead code and a latent silently-dropped sensor-stop bug; and within ten sprints another phased migration (055–061) restructured much of this freshly-built architecture again — the "migration complete" declaration in 044-004 held roughly ten days before the next re-seaming began.
