# KAYN Controller — Structured Logging & Acados Availability Check

**Date:** 2026-05-13  
**Status:** Approved

---

## Summary

Add the same structured logging style used in `race_monitor` to `kayn_controller`, and add an acados availability check at node startup. If acados is unavailable and the curve controller is configured as `mpc`, emit a critical warning and automatically reconfigure to the fallback controller before the control loop starts.

---

## Architecture

Four files are touched; one new file is created.

```
kayn_controller/
  kayn_controller/
    logger_utils.py        ← NEW: KAYNLogger (mirrors RaceMonitorLogger)
    kayn_node.py           ← MODIFIED: use KAYNLogger + acados startup check
    controllers/
      mpc.py               ← MODIFIED: wrap acados import, expose ACADOS_AVAILABLE
    supervisor/
      fsm.py               ← MODIFIED: accept KAYNLogger, use structured transition logs
```

---

## Components

### 1. `logger_utils.py` (new)

A self-contained `KAYNLogger` class that wraps the ROS2 node logger with:

- `LogLevel` enum: `MINIMAL | NORMAL | DEBUG | VERBOSE` (same ordering as race_monitor)
- Level-gated methods: `info`, `warn`, `debug`, `verbose`
- Always-on methods: `error`, `critical`, `startup`, `shutdown`
- Specialised methods: `event`, `metric`, `status`, `success`, `config`
- Message format: `[ComponentName] <prefix emoji> <message>`

Component names used in this codebase: `"KAYNNode"`, `"FSM"`.

No external dependencies beyond `rclpy`.

---

### 2. `mpc.py` — acados availability flag

Wrap the top-level acados/casadi imports in a `try/except ImportError`:

```python
try:
    from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
    from casadi import MX, vertcat, cos, sin, tan
    ACADOS_AVAILABLE = True
except ImportError:
    ACADOS_AVAILABLE = False
```

`MPCController.__init__` raises `RuntimeError("acados not available")` if `ACADOS_AVAILABLE` is `False`, so any accidental construction is caught early with a clear message rather than a cryptic attribute error.

`ACADOS_AVAILABLE` is the single source of truth — both `kayn_node.py` and `fsm.py` import it from here.

---

### 3. `kayn_node.py` — logger + startup acados check

**Logger instantiation** (early in `__init__`, before `_declare_params`):
```python
from .logger_utils import KAYNLogger, LogLevel
self._log = KAYNLogger(self, "KAYNNode")
```

**Acados check** (in `_load_params`, after reading `curve_ctrl`):
```python
from .controllers.mpc import ACADOS_AVAILABLE
if not ACADOS_AVAILABLE and self.curve_ctrl == 'mpc':
    self._log.critical("acados is not installed — MPC controller unavailable")
    self._log.warn(
        f"MPC skipped — curve controller falling back to '{self.fallback_ctrl}'"
    )
    self.curve_ctrl = self.fallback_ctrl
```

The swap happens before `_build_controllers()` is called, so the FSM is built with the correct curve controller from the start. No MPC solver is constructed when acados is absent.

**Existing log calls** replaced:

| Old call | New call |
|---|---|
| `self.get_logger().info(f"KAYN ready ...")` | `self._log.startup(...)` |
| `self.get_logger().error(f"odom_cb: {e}")` | `self._log.error(..., exception=e)` |
| `self.get_logger().info(f"Path ready: ...")` | `self._log.event("path_ready", ...)` |
| `self.get_logger().info(f"[{iter}] mode=...")` | `self._log.info(...)` with `LogLevel.DEBUG` |
| `self.get_logger().warning(f"KAYN blocked: {reason}")` | `self._log.warn(...)` |
| `self.get_logger().error(f"control_cb: {e}")` | `self._log.error(..., exception=e)` |

---

### 4. `fsm.py` — structured transition logging

**Constructor change:** `log_fn` parameter is replaced by `logger` that accepts either a `KAYNLogger` instance or a plain callable (for backwards compatibility with tests that pass `print`):

```python
# In FSM.__init__:
if callable(logger) and not hasattr(logger, 'event'):
    # plain callable (e.g. print, test stub) — wrap it
    class _Compat:
        def event(self, name, details='', level=None): logger(f"[KAYN] {name} | {details}")
        def warn(self, msg, level=None): logger(f"[KAYN] WARNING: {msg}")
    self._log = _Compat()
else:
    self._log = logger
```

**`_transition` method** — replace bare `self._log_fn(...)` with:
```python
self._log.event(
    f"{self.state.name} → {new_state.name}",
    details=f"{reason} | idx={ref_idx}"
)
```

**MPC fallback trigger** (`_step_curve`) — add a dedicated warning:
```python
self._log.warn(
    f"MPC skipped: {reason} — fallback controller '{self._fallback_ctrl}' taking over"
)
```

---

## Data Flow

```
kayn_node.__init__
  │
  ├─ KAYNLogger("KAYNNode") constructed
  ├─ params loaded
  ├─ ACADOS_AVAILABLE checked
  │    └─ if False + curve_ctrl=='mpc':
  │         critical() + warn() logged
  │         curve_ctrl ← fallback_ctrl
  │
  ├─ _build_controllers()
  │    └─ FSM constructed with KAYNLogger("FSM") + correct curve_ctrl
  │
  └─ control loop starts — no MPC ever constructed if acados absent
```

---

## Error Handling

- `ACADOS_AVAILABLE = False`: caught at startup, reconfigured before FSM build — no runtime crash.
- Acados libs present but solver construction fails (e.g. corrupted build): `MPCController.__init__` raises `RuntimeError`; caught in `_build_controllers` with `self._log.critical(...)`.
- MPC timeout / infeasible solve at runtime: existing FSM FALLBACK transition unchanged, now logs via `self._log.warn(...)` with the structured prefix.

---

## Testing

No new test files. Existing tests in `tests/test_mpc.py`, `tests/test_fsm.py` are unaffected because:
- `ACADOS_AVAILABLE` is module-level and importable for mocking.
- FSM `log_fn` backwards compatibility wrapper keeps `print`-based test stubs working.

---

## Out of Scope

- Adding `KAYNLogger` to `lqr.py`, `stanley.py`, `bicycle_model.py` — those are pure-math modules with no logging.
- Changing the `log_level` parameter — defaulting to `"normal"` matches race_monitor convention.
- Any changes to the FSM state machine logic itself.
