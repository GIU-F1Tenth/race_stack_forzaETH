# KAYN Controller Structured Logging & Acados Availability Check — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `RaceMonitorLogger`-style structured logging to `kayn_controller` and check acados availability at node startup, warning and falling back to the configured fallback controller if MPC is unavailable.

**Architecture:** A new `KAYNLogger` class (mirroring `RaceMonitorLogger`) is added to `kayn_controller/logger_utils.py`. `mpc.py` gains an `ACADOS_AVAILABLE` flag by wrapping its top-level imports in `try/except`. `kayn_node.py` uses `KAYNLogger` for all logging and checks `ACADOS_AVAILABLE` after loading params, reconfiguring `curve_ctrl` before the FSM is built if needed. `fsm.py` replaces its `log_fn` callable with a `logger` parameter that accepts either a `KAYNLogger` or a plain callable.

**Tech Stack:** Python 3.10, ROS2 Humble, pytest, unittest.mock

---

## File Map

| File | Action | Responsibility |
|---|---|---|
| `kayn_controller/logger_utils.py` | **Create** | `KAYNLogger` + `LogLevel` enum |
| `kayn_controller/controllers/mpc.py` | **Modify** | Wrap acados imports, expose `ACADOS_AVAILABLE`, guard `__init__` |
| `kayn_controller/supervisor/fsm.py` | **Modify** | `log_fn` → `logger`, compat wrapper, structured transition logs |
| `kayn_controller/kayn_node.py` | **Modify** | Use `KAYNLogger`, acados startup check, update all log calls |
| `tests/test_logger_utils.py` | **Create** | `KAYNLogger` unit tests with mock node |
| `tests/test_mpc.py` | **Modify** | Add `ACADOS_AVAILABLE` test, skip real-solver tests when unavailable |
| `tests/test_fsm.py` | **Modify** | Add `MockMPC`, use `logger=` param, add transition logging test |

All paths are relative to `master/kayn_controller/`.

---

## Task 1: Wrap acados imports in `mpc.py` — expose `ACADOS_AVAILABLE`

**Why first:** The current hard `from acados_template import ...` at module level crashes all test collection on machines without acados. This task restores test runnability and is a prerequisite for all others.

**Files:**
- Modify: `kayn_controller/controllers/mpc.py:19-42`
- Modify: `tests/test_mpc.py`

---

- [ ] **Step 1.1: Add `test_acados_available_is_bool` to `tests/test_mpc.py`**

Open `tests/test_mpc.py` and add this import and test at the top (before existing tests):

```python
from kayn_controller.controllers.mpc import ACADOS_AVAILABLE

def test_acados_available_is_bool():
    """ACADOS_AVAILABLE must be a bool regardless of whether acados is installed."""
    assert isinstance(ACADOS_AVAILABLE, bool)
```

- [ ] **Step 1.2: Run the new test — verify it fails (ACADOS_AVAILABLE not yet defined)**

```bash
cd master/kayn_controller && python3 -m pytest tests/test_mpc.py::test_acados_available_is_bool -v 2>&1 | tail -15
```

Expected: `ERROR` — `ModuleNotFoundError: No module named 'acados_template'`

- [ ] **Step 1.3: Wrap acados imports in `mpc.py` and expose `ACADOS_AVAILABLE`**

In `kayn_controller/controllers/mpc.py`, replace lines 41–43 (the hard imports):

```python
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from casadi import MX, vertcat, cos, sin, tan
```

with:

```python
try:
    from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
    from casadi import MX, vertcat, cos, sin, tan
    ACADOS_AVAILABLE = True
except ImportError:
    ACADOS_AVAILABLE = False
```

Then, in `MPCController.__init__`, add a guard as the very first line of the body:

```python
def __init__(self, model: BicycleModel,
             N: int = 15,
             dt: float = None,
             Q: np.ndarray = None,
             R: np.ndarray = None,
             v_max: float = None):
    if not ACADOS_AVAILABLE:
        raise RuntimeError(
            "acados is not installed — MPCController cannot be constructed. "
            "Set fsm.curve_controller to 'lqr' or 'stanley' in kayn_params.yaml."
        )
    self.model = model
    # ... rest of __init__ unchanged
```

- [ ] **Step 1.4: Add skip markers to real-solver tests in `test_mpc.py`**

At the top of `tests/test_mpc.py`, after the `ACADOS_AVAILABLE` import, add a module-level skip marker:

```python
import pytest
from kayn_controller.controllers.mpc import ACADOS_AVAILABLE

pytestmark = pytest.mark.skipif(
    not ACADOS_AVAILABLE,
    reason="acados not installed"
)

def test_acados_available_is_bool():
    """ACADOS_AVAILABLE must be a bool regardless of whether acados is installed."""
    assert isinstance(ACADOS_AVAILABLE, bool)
```

Note: `pytestmark` applies to all tests in the file *except* `test_acados_available_is_bool` because pytest evaluates module-level skip at collection, but the test itself only does an `isinstance` check that doesn't require acados. To make `test_acados_available_is_bool` run regardless, move it to a separate file or override with a `@pytest.mark.skipif(False, ...)`. The cleanest solution: remove `pytestmark` and instead decorate each acados-dependent test:

```python
import pytest
import numpy as np
import time
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from kayn_controller.controllers.mpc import MPCController, ACADOS_AVAILABLE
from simulation.track import curve_track, straight_track

_needs_acados = pytest.mark.skipif(not ACADOS_AVAILABLE, reason="acados not installed")


def test_acados_available_is_bool():
    assert isinstance(ACADOS_AVAILABLE, bool)


@_needs_acados
def test_ocp_setup_no_error():
    model = BicycleModel()
    mpc = MPCController(model)
    assert mpc.solver is not None


@_needs_acados
def test_feasible_on_curve():
    model = BicycleModel()
    mpc = MPCController(model)
    track = curve_track(radius=3.0, sweep_deg=90.0, v_ref=2.0, n_points=100)
    x_curr = np.array([track[0]['x'], track[0]['y'], track[0]['theta'], 2.0])
    ref_traj = track[:mpc.N + 1]
    u, solve_time, status = mpc.compute_control(x_curr, ref_traj)
    assert status == 0, f"MPC infeasible (status={status})"
    assert u.shape == (2,)


@_needs_acados
def test_solve_time_within_budget():
    model = BicycleModel()
    mpc = MPCController(model)
    track = curve_track(radius=3.0, sweep_deg=90.0, v_ref=2.0, n_points=100)
    x_curr = np.array([track[0]['x'], track[0]['y'], track[0]['theta'], 2.0])
    ref_traj = track[:mpc.N + 1]
    mpc.compute_control(x_curr, ref_traj)
    _, solve_time, _ = mpc.compute_control(x_curr, ref_traj)
    assert solve_time < 0.010, f"MPC too slow: {solve_time*1000:.1f}ms"


@_needs_acados
def test_steering_output_within_limits():
    model = BicycleModel()
    mpc = MPCController(model)
    track = curve_track(radius=3.0, sweep_deg=90.0, v_ref=2.0, n_points=100)
    x_curr = np.array([track[0]['x'], track[0]['y'], track[0]['theta'], 2.0])
    ref_traj = track[:mpc.N + 1]
    u, _, _ = mpc.compute_control(x_curr, ref_traj)
    assert abs(u[0]) <= 0.4189 + 1e-6, f"delta={u[0]:.4f} exceeds limit"


@_needs_acados
def test_acados_matches_intuition_on_straight():
    model = BicycleModel()
    mpc = MPCController(model)
    track = straight_track(length=50.0, v_ref=2.0, n_points=200)
    x_curr = np.array([0.0, 0.0, 0.0, 2.0])
    ref_traj = track[:mpc.N + 1]
    u, _, status = mpc.compute_control(x_curr, ref_traj)
    assert status == 0
    assert abs(u[0]) < 0.05, f"Expected near-zero steering on straight, got {u[0]:.4f}"


@_needs_acados
def test_heading_normalization_near_pi():
    model = BicycleModel()
    mpc = MPCController(model)
    track = []
    for i in range(40):
        theta = np.pi if i % 2 == 0 else -np.pi
        track.append({'x': float(-i) * 0.3, 'y': 0.0, 'theta': theta, 'v': 2.0})
    x_curr = np.array([0.0, 0.0, np.pi, 2.0])
    u, _, status = mpc.compute_control(x_curr, track)
    assert status == 0, f"MPC should be feasible near ±π heading (status={status})"
    assert abs(u[0]) < 0.05, f"Expected near-zero steering near ±π, got delta={u[0]:.4f}"


@_needs_acados
def test_mpc_defaults_match_yaml():
    model = BicycleModel()
    mpc = MPCController(model)
    assert mpc.Q[0, 0] == 7.0
    assert mpc.Q[2, 2] == 5.0
    assert mpc.Q[3, 3] == 9.0
    assert mpc.R[0, 0] == 8.0
    assert mpc.R[1, 1] == 0.3
```

Note: also add `from kayn_controller.controllers.bicycle_model import BicycleModel` back to the test file header since the original had it via the existing `from kayn_controller.controllers.mpc import MPCController` import path. The full header for `test_mpc.py` should be:

```python
import numpy as np
import time
import sys, os
import pytest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from kayn_controller.controllers.bicycle_model import BicycleModel
from kayn_controller.controllers.mpc import MPCController, ACADOS_AVAILABLE
from simulation.track import curve_track, straight_track

_needs_acados = pytest.mark.skipif(not ACADOS_AVAILABLE, reason="acados not installed")
```

- [ ] **Step 1.5: Add `MockMPC` and fix `_make_fsm` in `tests/test_fsm.py`**

At the top of `tests/test_fsm.py`, after existing imports, add:

```python
from kayn_controller.controllers.mpc import ACADOS_AVAILABLE


class MockMPC:
    """Drop-in MPC stub for tests that don't need real acados."""
    def compute_control(self, x, traj):
        return np.zeros(2), 0.001, 0
```

Replace the existing `_make_fsm()` function with:

```python
def _make_fsm(**kwargs):
    model = BicycleModel()
    mpc = MockMPC() if not ACADOS_AVAILABLE else MPCController(model)
    return FSM(
        lqr=LQRController(model),
        mpc=mpc,
        stanley=StanleyController(model=model),
        curvature_estimator=CurvatureEstimator(lookahead=10),
        **kwargs,
    )
```

Also update `test_curve_slot_lqr_no_fallback` and `test_fallback_on_mpc_timeout` to use `MockMPC` directly (they monkeypatch `mpc.compute_control` anyway so the real solver is not needed):

```python
def test_curve_slot_lqr_no_fallback(monkeypatch):
    model = BicycleModel()
    fsm = FSM(
        lqr=LQRController(model),
        mpc=MockMPC(),
        stanley=StanleyController(model=model),
        curvature_estimator=CurvatureEstimator(lookahead=10),
        curve_ctrl='lqr',
    )
    fsm.state = KAYNState.CURVE
    track = curve_track(radius=3.0, sweep_deg=180.0, v_ref=2.0, n_points=300)
    x_curr = np.array([track[10]['x'], track[10]['y'], track[10]['theta'], 2.0])
    monkeypatch.setattr(fsm.mpc, 'compute_control',
                        lambda *a, **kw: (np.zeros(2), 0.010, 0))
    fsm.step(x_curr, track, 10)
    assert fsm.state != KAYNState.FALLBACK


def test_fallback_on_mpc_timeout(monkeypatch):
    fsm = _make_fsm()
    track = curve_track(radius=3.0, sweep_deg=180.0, v_ref=2.0, n_points=300)
    x_curr = np.array([track[10]['x'], track[10]['y'], track[10]['theta'], 2.0])
    fsm.state = KAYNState.CURVE
    monkeypatch.setattr(fsm.mpc, 'compute_control',
                        lambda *a, **kw: (np.zeros(2), 0.010, 0))
    fsm.step(x_curr, track, 10)
    assert fsm.state == KAYNState.FALLBACK


def test_blend_in_mpc_infeasible_uses_lqr(monkeypatch):
    fsm = _make_fsm()
    track = straight_track(length=50.0, v_ref=2.0, n_points=100)
    x_curr = np.array([track[5]['x'], track[5]['y'], track[5]['theta'], 2.0])
    fsm.state = KAYNState.BLEND_IN
    fsm._blend_step = 2
    monkeypatch.setattr(fsm.mpc, 'compute_control',
                        lambda x, traj: (np.array([0.9, 2.0]), 0.001, 1))
    u = fsm.step(x_curr, track, 5)
    assert abs(u[0]) < 0.5
    assert fsm.state == KAYNState.STRAIGHT
```

- [ ] **Step 1.6: Run all tests — verify collection no longer fails**

```bash
cd master/kayn_controller && python3 -m pytest tests/ -v 2>&1 | tail -25
```

Expected: tests collect and run; acados-dependent tests show `SKIPPED` (not `ERROR`); `test_acados_available_is_bool` and FSM tests `PASS`.

- [ ] **Step 1.7: Commit**

```bash
cd master/kayn_controller && git add kayn_controller/controllers/mpc.py tests/test_mpc.py tests/test_fsm.py
git commit -m "fix: wrap acados imports — expose ACADOS_AVAILABLE, skip tests when unavailable"
```

---

## Task 2: Create `kayn_controller/logger_utils.py`

**Files:**
- Create: `kayn_controller/logger_utils.py`
- Create: `tests/test_logger_utils.py`

---

- [ ] **Step 2.1: Write failing tests in `tests/test_logger_utils.py`**

```python
import sys, os
import pytest
from unittest.mock import MagicMock
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from kayn_controller.logger_utils import KAYNLogger, LogLevel


def _make_logger(component="Comp", level="normal"):
    node = MagicMock()
    ros_log = MagicMock()
    node.get_logger.return_value = ros_log
    return KAYNLogger(node, component, level), ros_log


def test_info_includes_component_tag():
    kl, ros_log = _make_logger("MyComp")
    kl.info("hello")
    ros_log.info.assert_called_once()
    msg = ros_log.info.call_args[0][0]
    assert "[MyComp]" in msg
    assert "hello" in msg


def test_warn_includes_warning_prefix():
    kl, ros_log = _make_logger()
    kl.warn("something wrong")
    ros_log.warn.assert_called_once()
    msg = ros_log.warn.call_args[0][0]
    assert "WARNING" in msg or "⚠" in msg
    assert "something wrong" in msg


def test_error_includes_exception_text():
    kl, ros_log = _make_logger()
    kl.error("bad thing", exception=ValueError("boom"))
    ros_log.error.assert_called_once()
    msg = ros_log.error.call_args[0][0]
    assert "boom" in msg


def test_critical_uses_error_channel():
    kl, ros_log = _make_logger()
    kl.critical("system failure")
    ros_log.error.assert_called_once()


def test_startup_always_logs():
    kl, ros_log = _make_logger(level="minimal")
    kl.startup("node online")
    ros_log.info.assert_called_once()
    msg = ros_log.info.call_args[0][0]
    assert "node online" in msg


def test_debug_suppressed_at_normal_level():
    kl, ros_log = _make_logger(level="normal")
    kl.debug("verbose detail")
    ros_log.info.assert_not_called()


def test_debug_visible_at_debug_level():
    kl, ros_log = _make_logger(level="debug")
    kl.debug("verbose detail")
    ros_log.info.assert_called_once()


def test_event_includes_name_and_details():
    kl, ros_log = _make_logger()
    kl.event("lap_start", details="lap=3")
    ros_log.info.assert_called_once()
    msg = ros_log.info.call_args[0][0]
    assert "lap_start" in msg
    assert "lap=3" in msg


def test_invalid_log_level_defaults_to_normal():
    kl, ros_log = _make_logger(level="nonsense")
    assert kl.log_level == LogLevel.NORMAL
```

- [ ] **Step 2.2: Run — verify they fail (module not found)**

```bash
cd master/kayn_controller && python3 -m pytest tests/test_logger_utils.py -v 2>&1 | tail -10
```

Expected: `ModuleNotFoundError: No module named 'kayn_controller.logger_utils'`

- [ ] **Step 2.3: Create `kayn_controller/logger_utils.py`**

```python
from enum import Enum
from typing import Optional
from rclpy.node import Node


class LogLevel(Enum):
    MINIMAL = "minimal"
    NORMAL  = "normal"
    DEBUG   = "debug"
    VERBOSE = "verbose"


_LEVEL_ORDER = {
    LogLevel.MINIMAL: 0,
    LogLevel.NORMAL:  1,
    LogLevel.DEBUG:   2,
    LogLevel.VERBOSE: 3,
}


class KAYNLogger:
    """Structured logging wrapper for KAYN controller components."""

    def __init__(self, node: Node, component_name: str, log_level: str = "normal"):
        self.node = node
        self.component = component_name
        self._set_log_level(log_level)

    def _set_log_level(self, level: str):
        try:
            self.log_level = LogLevel(level.lower())
        except ValueError:
            self.node.get_logger().warn(
                f"[{self.component}] Invalid log level '{level}'. Using 'normal'."
            )
            self.log_level = LogLevel.NORMAL

    def _fmt(self, message: str, prefix: str = "") -> str:
        tag = f"[{self.component}]"
        return f"{tag} {prefix} {message}" if prefix else f"{tag} {message}"

    def _should_log(self, required: LogLevel) -> bool:
        return _LEVEL_ORDER[self.log_level] >= _LEVEL_ORDER[required]

    # ── always-on ────────────────────────────────────────────────────────────

    def error(self, message: str, exception: Optional[Exception] = None):
        text = self._fmt(message, "❌ ERROR:")
        if exception:
            text += f" | Exception: {exception}"
        self.node.get_logger().error(text)

    def critical(self, message: str):
        self.node.get_logger().error(self._fmt(message, "🚨 CRITICAL:"))

    def startup(self, message: str):
        self.node.get_logger().info(self._fmt(message, "STARTUP:"))

    def shutdown(self, message: str):
        self.node.get_logger().info(self._fmt(message, "🛑 SHUTDOWN:"))

    # ── level-gated ──────────────────────────────────────────────────────────

    def warn(self, message: str, level: LogLevel = LogLevel.NORMAL):
        if self._should_log(level):
            self.node.get_logger().warn(self._fmt(message, "⚠️  WARNING:"))

    def info(self, message: str, level: LogLevel = LogLevel.NORMAL):
        if self._should_log(level):
            self.node.get_logger().info(self._fmt(message))

    def success(self, message: str, level: LogLevel = LogLevel.NORMAL):
        if self._should_log(level):
            self.node.get_logger().info(self._fmt(message, "✓"))

    def debug(self, message: str):
        if self._should_log(LogLevel.DEBUG):
            self.node.get_logger().info(self._fmt(message, "🔍 DEBUG:"))

    def verbose(self, message: str):
        if self._should_log(LogLevel.VERBOSE):
            self.node.get_logger().info(self._fmt(message, "📝 VERBOSE:"))

    # ── specialised ──────────────────────────────────────────────────────────

    def event(self, event_name: str, details: str = "", level: LogLevel = LogLevel.NORMAL):
        if self._should_log(level):
            msg = f"📍 EVENT: {event_name}"
            if details:
                msg += f" | {details}"
            self.node.get_logger().info(self._fmt(msg))

    def metric(self, name: str, value, unit: str = "", level: LogLevel = LogLevel.NORMAL):
        if self._should_log(level):
            val_str = f"{value:.4f}" if isinstance(value, float) else str(value)
            msg = f"-> {name}: {val_str}"
            if unit:
                msg += f" {unit}"
            self.node.get_logger().info(self._fmt(msg))

    def status(self, status: str, level: LogLevel = LogLevel.NORMAL):
        if self._should_log(level):
            self.node.get_logger().info(self._fmt(status, "🔄 STATUS:"))

    def config(self, parameter: str, value, level: LogLevel = LogLevel.DEBUG):
        if self._should_log(level):
            self.node.get_logger().info(self._fmt(f"⚙️  CONFIG: {parameter} = {value}"))
```

- [ ] **Step 2.4: Run tests — verify they pass**

```bash
cd master/kayn_controller && python3 -m pytest tests/test_logger_utils.py -v 2>&1 | tail -20
```

Expected: all 9 tests `PASS`.

- [ ] **Step 2.5: Commit**

```bash
cd master/kayn_controller && git add kayn_controller/logger_utils.py tests/test_logger_utils.py
git commit -m "feat: add KAYNLogger — structured logging utility for kayn_controller"
```

---

## Task 3: Update `fsm.py` to use `KAYNLogger`

**Files:**
- Modify: `kayn_controller/supervisor/fsm.py`
- Modify: `tests/test_fsm.py`

---

- [ ] **Step 3.1: Add transition-logging test to `tests/test_fsm.py`**

Add this test at the bottom of `tests/test_fsm.py`:

```python
def test_fsm_logs_transition_via_event():
    """FSM must call logger.event() on state transitions."""
    events = []

    class _Logger:
        def event(self, name, details='', level=None):
            events.append(name)
        def warn(self, msg, level=None):
            pass

    model = BicycleModel()
    fsm = FSM(
        lqr=LQRController(model),
        mpc=MockMPC(),
        stanley=StanleyController(model=model),
        curvature_estimator=CurvatureEstimator(lookahead=10),
        logger=_Logger(),
    )
    track = straight_track(length=200.0, v_ref=2.0, n_points=300)
    x_curr = np.array([track[5]['x'], track[5]['y'], track[5]['theta'], 2.0])
    for _ in range(WARMUP_STEPS):
        fsm.step(x_curr, track, 5)

    assert any("WARMUP" in e and "STRAIGHT" in e for e in events), \
        f"Expected WARMUP → STRAIGHT transition event. Got: {events}"


def test_fsm_plain_callable_logger_still_works():
    """Passing a plain callable (e.g. print) as logger must not crash."""
    calls = []
    fsm = _make_fsm(logger=calls.append)
    track = straight_track(length=200.0, v_ref=2.0, n_points=300)
    x_curr = np.array([track[5]['x'], track[5]['y'], track[5]['theta'], 2.0])
    for _ in range(WARMUP_STEPS):
        fsm.step(x_curr, track, 5)
    assert len(calls) > 0, "Transition must produce at least one log entry"
```

- [ ] **Step 3.2: Run — verify the new tests fail**

```bash
cd master/kayn_controller && python3 -m pytest tests/test_fsm.py::test_fsm_logs_transition_via_event tests/test_fsm.py::test_fsm_plain_callable_logger_still_works -v 2>&1 | tail -15
```

Expected: `FAILED` — `FSM.__init__() got an unexpected keyword argument 'logger'`

- [ ] **Step 3.3: Update `fsm.py` — replace `log_fn` with `logger`, add compat wrapper, update `_transition` and fallback warn**

In `kayn_controller/supervisor/fsm.py`:

**3.3a** — Change the constructor signature (line 68): replace `log_fn=print` with `logger=print`:

```python
def __init__(self, lqr, mpc, stanley, curvature_estimator: CurvatureEstimator,
             warmup_steps: int = WARMUP_STEPS,
             warmup_ctrl: str = 'stanley',
             straight_ctrl: str = 'lqr',
             curve_ctrl: str = 'mpc',
             fallback_ctrl: str = 'stanley',
             confirm_steps: int = CONFIRM_STEPS,
             blend_window: int = BLEND_WINDOW,
             mpc_timeout_s: float = MPC_TIMEOUT_S,
             v_warmup_min: float = 0.2,
             v_stop: float = 0.05,
             stop_confirm_steps: int = 20,
             dt: float = 0.005,
             logger=print):
```

**3.3b** — Replace the `self._log_fn = log_fn` assignment (last line of `__init__`) with the compat wrapper:

```python
        if callable(logger) and not hasattr(logger, 'event'):
            _fn = logger
            class _Compat:
                def event(self, name, details='', level=None):
                    _fn(f"[KAYN] {name} | {details}")
                def warn(self, msg, level=None):
                    _fn(f"[KAYN] WARNING: {msg}")
            self._log = _Compat()
        else:
            self._log = logger
```

**3.3c** — Update `_transition` method — replace the `self._log_fn(...)` call:

```python
    def _transition(self, new_state: KAYNState, reason: str,
                    x_curr: np.ndarray, trajectory: List[Dict],
                    ref_idx: int) -> None:
        self._log.event(
            f"{self.state.name} → {new_state.name}",
            details=f"{reason} | idx={ref_idx}",
        )
        self.state = new_state
        self._confirm_count  = 0
        self._blend_step     = 0
        self._recovery_count = 0
        self._stop_count     = 0
        if new_state == KAYNState.WARMUP:
            self._warmup_count = 0
            self._prev_pos = None

        if new_state in (KAYNState.CURVE, KAYNState.BLEND_OUT):
            handoff(self.mpc, x_curr, trajectory, ref_idx)
        elif new_state in (KAYNState.STRAIGHT, KAYNState.BLEND_IN):
            handoff(self.lqr, x_curr, trajectory, ref_idx)
```

**3.3d** — In `_step_curve`, add a dedicated `warn` before the fallback `_transition`. The relevant block (around line 181):

```python
        if self._curve_ctrl == 'mpc' and (status != 0 or solve_time > self._mpc_timeout_s):
            reason = (f"solver_timeout={solve_time*1000:.1f}ms"
                      if solve_time > self._mpc_timeout_s else f"infeasible status={status}")
            self._log.warn(
                f"MPC skipped: {reason} — fallback controller '{self._fallback_ctrl}' taking over"
            )
            self._transition(KAYNState.FALLBACK, reason, x_curr, trajectory, ref_idx)
            u_fb, _, _ = self._ctrl_u(self._fallback_ctrl, x_curr, trajectory, ref_idx)
            return u_fb
```

**3.3e** — In `_step_blend_in`, add a warn before the STRAIGHT transition (around line 202):

```python
        if self._curve_ctrl == 'mpc' and (status != 0 or solve_time > self._mpc_timeout_s):
            self._log.warn(
                f"MPC infeasible during BLEND_IN (status={status}, t={solve_time*1000:.1f}ms) "
                f"— transitioning to STRAIGHT"
            )
            self._transition(KAYNState.STRAIGHT, "blend_in_mpc_infeasible",
                             x_curr, trajectory, ref_idx)
            return u_in
```

- [ ] **Step 3.4: Run FSM tests — verify they all pass**

```bash
cd master/kayn_controller && python3 -m pytest tests/test_fsm.py -v 2>&1 | tail -25
```

Expected: all tests `PASS` (no `ERROR`, no `FAILED`).

- [ ] **Step 3.5: Commit**

```bash
cd master/kayn_controller && git add kayn_controller/supervisor/fsm.py tests/test_fsm.py
git commit -m "feat: FSM uses KAYNLogger for structured transition and fallback logging"
```

---

## Task 4: Update `kayn_node.py` — `KAYNLogger` + acados startup check

**Files:**
- Modify: `kayn_controller/kayn_node.py`

No new test file — `kayn_node.py` is a ROS2 node; existing test suite covers behaviour. Verify by running all tests after the change.

---

- [ ] **Step 4.1: Add `KAYNLogger` import and `ACADOS_AVAILABLE` import to top of `kayn_node.py`**

After the existing local imports block (around line 39-46), add:

```python
from .logger_utils import KAYNLogger, LogLevel
from .controllers.mpc import MPCController, ACADOS_AVAILABLE
```

Remove the existing standalone `from .controllers.mpc import MPCController` (it's now included above).

- [ ] **Step 4.2: Add `_NullMPC` sentinel class just before `class KAYNNode`**

Insert this after imports, before `_CURVE_STATES = ...`:

```python
class _NullMPC:
    """Placeholder used when acados is unavailable; FSM never calls it after reconfiguration."""
    def compute_control(self, *args, **kwargs):
        raise RuntimeError("acados unavailable — MPC must not be called")
```

- [ ] **Step 4.3: Instantiate `KAYNLogger` as the first action in `__init__`**

Replace the existing `__init__` body opening:

```python
def __init__(self):
    super().__init__('kayn_controller_node')
    self._declare_params()
    self._load_params()
```

with:

```python
def __init__(self):
    super().__init__('kayn_controller_node')
    self._log = KAYNLogger(self, "KAYNNode")
    self._declare_params()
    self._load_params()
```

- [ ] **Step 4.4: Replace the startup `get_logger().info` at the end of `__init__`**

Replace:

```python
        self.get_logger().info(
            f"KAYN ready | {self.control_hz}Hz | "
            f"warmup={self.warmup_ctrl} straight={self.straight_ctrl} "
            f"curve={self.curve_ctrl} fallback={self.fallback_ctrl}"
        )
```

with:

```python
        self._log.startup(
            f"KAYN ready | {self.control_hz}Hz | "
            f"warmup={self.warmup_ctrl} straight={self.straight_ctrl} "
            f"curve={self.curve_ctrl} fallback={self.fallback_ctrl}"
        )
```

- [ ] **Step 4.5: Add acados check at the end of `_load_params`**

After the last assignment in `_load_params` (currently `self.curve_speed_scale = ...`), add:

```python
        if not ACADOS_AVAILABLE and self.curve_ctrl == 'mpc':
            self._log.critical("acados is not installed — MPC controller unavailable")
            self._log.warn(
                f"MPC skipped — curve controller falling back to '{self.fallback_ctrl}'"
            )
            self.curve_ctrl = self.fallback_ctrl
```

- [ ] **Step 4.6: Update `_build_controllers` to use `KAYNLogger("FSM")` and conditional MPC**

Replace the entire `_build_controllers` method:

```python
    def _build_controllers(self):
        model = BicycleModel(L=self.wheelbase, dt=self.dt,
                             delta_max=self.max_steering,
                             a_max=self.max_accel,
                             v_max=self.max_speed)
        curv_est = CurvatureEstimator(
            lookahead=self.curv_lookahead,
            enter_threshold=self.enter_threshold,
            exit_threshold=self.exit_threshold,
        )
        mpc_ctrl = (
            MPCController(model, N=self.mpc_n, dt=self.mpc_dt,
                          Q=self.mpc_Q, R=self.mpc_R, v_max=self.max_speed)
            if ACADOS_AVAILABLE
            else _NullMPC()
        )
        self.fsm = FSM(
            lqr=LQRController(model, Q=self.lqr_Q, R=self.lqr_R),
            mpc=mpc_ctrl,
            stanley=StanleyController(k=self.stanley_k, model=model),
            curvature_estimator=curv_est,
            warmup_steps=self.warmup_steps,
            warmup_ctrl=self.warmup_ctrl,
            straight_ctrl=self.straight_ctrl,
            curve_ctrl=self.curve_ctrl,
            fallback_ctrl=self.fallback_ctrl,
            confirm_steps=self.confirm_steps,
            blend_window=self.blend_window,
            mpc_timeout_s=self.mpc_timeout_s,
            v_warmup_min=self.v_warmup_min,
            v_stop=self.v_stop,
            stop_confirm_steps=self.stop_confirm_steps,
            dt=self.dt,
            logger=KAYNLogger(self, "FSM"),
        )
```

- [ ] **Step 4.7: Replace remaining `get_logger()` calls**

**`_odom_cb`** — replace:
```python
            self.get_logger().error(f"odom_cb: {e}")
```
with:
```python
            self._log.error("odom_cb", exception=e)
```

**`_ready_cb`** — replace:
```python
            self.get_logger().info(f"Path ready: {self.path_ready}")
```
with:
```python
            self._log.event("path_ready", f"ready={self.path_ready}")
```

**`_control_cb` — periodic debug log** — replace:
```python
            if self.debug or self._iter % self.log_every_n == 0:
                self.get_logger().info(
                    f"[{self._iter}] mode={self.fsm.state_name} "
                    f"v={self.x_curr[3]:.2f} steer={u[0]:.3f} "
                    f"kappa={kappa:.3f} cte={cte:.3f}"
                )
```
with:
```python
            if self.debug or self._iter % self.log_every_n == 0:
                self._log.info(
                    f"[{self._iter}] mode={self.fsm.state_name} "
                    f"v={self.x_curr[3]:.2f} steer={u[0]:.3f} "
                    f"kappa={kappa:.3f} cte={cte:.3f}",
                    LogLevel.DEBUG,
                )
```

**`_control_cb` — exception handler** — replace:
```python
        except Exception as e:
            self.get_logger().error(f"control_cb: {e}")
```
with:
```python
        except Exception as e:
            self._log.error("control_cb", exception=e)
```

**`_block`** — replace:
```python
            self.get_logger().warning(f"KAYN blocked: {reason}")
```
with:
```python
            self._log.warn(f"KAYN blocked: {reason}")
```

- [ ] **Step 4.8: Run full test suite — verify all tests pass**

```bash
cd master/kayn_controller && python3 -m pytest tests/ -v 2>&1 | tail -30
```

Expected: all non-acados tests `PASS`, acados tests `SKIPPED`. Zero `FAILED` or `ERROR`.

- [ ] **Step 4.9: Commit**

```bash
cd master/kayn_controller && git add kayn_controller/kayn_node.py
git commit -m "feat: kayn_node uses KAYNLogger and checks acados availability at startup"
```

---

## Done

After all four tasks:

- `ACADOS_AVAILABLE` is the single source of truth for whether acados is usable.
- Node startup emits a `🚨 CRITICAL` + `⚠️  WARNING` if acados is absent and `curve_ctrl='mpc'`.
- All controller logs carry `[KAYNNode]` or `[FSM]` component tags with emoji-prefixed severity.
- State transitions log via `logger.event(...)` with state-name arrows and reason.
- MPC timeout/infeasible events log via `logger.warn(...)` before transitioning to FALLBACK.
- Tests run cleanly on machines without acados.
