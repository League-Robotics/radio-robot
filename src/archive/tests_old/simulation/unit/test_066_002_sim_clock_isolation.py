"""
test_066_002_sim_clock_isolation.py — sim C-ABI clock cross-thread isolation
(ticket 066-002 / CR-13).

Background
----------
``g_sim_now_ms`` (tests/_infra/sim/sim_api.cpp) used to be a single
process-global ``uint32_t``. ``sim_create()`` resets it to 0. SimTransport's
real threading model is one OS thread per ``Sim()`` lifetime — a fresh thread
per ``connect()``, and ``disconnect()``'s ``join(timeout=3.0)`` can time out
leaving the previous tick-thread briefly alive. A GUI reconnect racing a
slow-exiting prior tick-thread would therefore call ``sim_create()`` on a
*different* thread while the first ``SimHandle`` is still live, resetting the
shared global to 0 out from under it — corrupting the still-live instance's
TIME-stop/watchdog deltas (a real command's ``t0Ms`` baseline reads whatever
``g_sim_now_ms`` happens to hold at that moment via ``robot->systemTime()``).

``g_sim_now_ms`` is now ``thread_local``: each OS thread gets its own
physically separate storage, so a second thread's ``sim_create()`` reset
cannot affect any other thread's value.

This test reproduces the exact race: thread A ticks its ``Sim`` forward,
then *pauses* (does not tick) while thread B constructs and uses its own
``Sim`` (calling ``sim_create()``, which resets thread B's own clock
storage). Thread A then issues a fresh TIMED command and must see it run its
full duration — if thread B's ``sim_create()`` had corrupted thread A's
clock (the pre-fix behaviour), the command would baseline against the
stale/reset value and fire instantly on thread A's next tick, exactly the
CR-11-shaped symptom the issue calls out ("corrupting watchdog/TIME-stop
deltas").
"""

from __future__ import annotations

import pathlib
import sys
import threading

_HERE = pathlib.Path(__file__).parent
_REPO = _HERE.parent.parent.parent
_SIM_DIR = _REPO / "tests" / "_infra" / "sim"

if str(_SIM_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_DIR))


def test_two_sequential_sim_instances_on_different_threads_do_not_corrupt_clock(build_lib):
    """A second Sim() constructed on a different thread must not yank the
    first (still-live) Sim's clock backwards.
    """
    # Imported here (after build_lib has run) — module-level import would
    # load the library before the session-scoped build fixture has run it.
    from firmware import Sim  # noqa: PLC0415

    sim1_ticked = threading.Event()   # thread A has advanced its clock; safe for B to start
    sim2_done = threading.Event()     # thread B has constructed+used its Sim and torn it down
    result: dict[str, object] = {}
    errors: list[str] = []

    def worker_a() -> None:
        try:
            with Sim() as s1:
                s1.send_command("SET sTimeout=60000")
                # Advance thread A's clock well past 0 first.
                s1.tick_for(5000)
                sim1_ticked.set()

                # Wait for thread B to construct (sim_create() resets ITS OWN
                # thread_local storage) and tear down its own Sim WHILE
                # thread A is paused (not ticking) — this is exactly the
                # vulnerable window under the pre-fix shared-global model.
                if not sim2_done.wait(timeout=10.0):
                    errors.append("timed out waiting for thread B")
                    return

                # Issue a fresh TIMED command on thread A. Its t0Ms baseline
                # is stamped from whatever g_sim_now_ms holds at this instant
                # (via robot->systemTime()) — corrupted under the old shared
                # -global bug (thread B's sim_create() reset it to 0 while
                # thread A was paused), correct under thread_local isolation
                # (thread A's storage was untouched by thread B).
                s1.send_command("T 100 100 300")  # 300 ms timed straight drive

                s1.tick_for(150)  # 150 ms < 300 ms duration — must NOT be done yet
                result["early_evts"] = s1.get_async_evts()

                # Well past the 300 ms duration AND the ~400 ms SOFT-ramp-down
                # convergence time (v=100mm/s, aDecel=250mm/s^2) that follows
                # it — comfortably inside the 3 s SOFT-stop deadline cap.
                s1.tick_for(1200)
                result["late_evts"] = s1.get_async_evts()
        except Exception as exc:  # pragma: no cover - surfaced via errors list
            errors.append(f"worker_a exception: {exc!r}")
            sim1_ticked.set()  # unblock worker_b so the test doesn't hang

    def worker_b() -> None:
        if not sim1_ticked.wait(timeout=10.0):
            errors.append("thread A never reached the wait point")
            sim2_done.set()
            return
        try:
            with Sim() as s2:
                s2.send_command("SET sTimeout=60000")
                s2.tick_for(50)
                result["sim2_ok"] = True
        except Exception as exc:  # pragma: no cover
            errors.append(f"worker_b exception: {exc!r}")
        finally:
            sim2_done.set()

    t_a = threading.Thread(target=worker_a, name="sim-clock-isolation-A")
    t_b = threading.Thread(target=worker_b, name="sim-clock-isolation-B")
    t_a.start()
    t_b.start()
    t_a.join(timeout=20.0)
    t_b.join(timeout=20.0)

    assert not errors, f"worker thread(s) reported errors: {errors}"
    assert result.get("sim2_ok") is True, "thread B's Sim never completed its ticks"

    assert "EVT done T" not in result.get("early_evts", ""), (
        "thread A's T command completed after only 150/300 ms — thread B's "
        "sim_create() appears to have corrupted thread A's clock baseline "
        "(CR-13 regression: g_sim_now_ms is no longer thread-isolated)."
    )
    assert "EVT done T" in result.get("late_evts", ""), (
        "thread A's T command never completed even after its full 300 ms "
        f"duration elapsed; late_evts={result.get('late_evts')!r}"
    )
