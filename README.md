# libpyflexiv

Python real-time control bindings for Flexiv robots (Rizon 4/4s/10/10s), built on [Flexiv RDK v1.8](https://github.com/flexivrobotics/flexiv_rdk) and [pybind11](https://github.com/pybind/pybind11).

## Why This Project Exists

Flexiv RDK requires **1 kHz SCHED_FIFO callbacks** for real-time streaming control (`StreamJointPosition`, `StreamCartesianMotionForce`). Python cannot meet this deadline due to the GIL and non-RT scheduling.

**libpyflexiv** solves this by running the 1 kHz RT loop entirely in C++ on a dedicated SCHED_FIFO thread, while Python sets targets and reads state through mutex-protected shared memory at any rate (typically 100 Hz).

```
Python (non-RT, ~100 Hz)              C++ RT thread (SCHED_FIFO, 1 kHz)
┌──────────────────────┐              ┌──────────────────────────────┐
│ set_target_pose()    │──── mutex ──►│ Read command from SHM        │
│ set_target_joints()  │              │ Safety checks (NaN, jump)    │
│ state = get_state()  │◄─── mutex ──│ Timeout detection (500ms)    │
│ move_to_pose()       │              │ Trajectory interpolation     │
└──────────────────────┘              │ StreamCartesianMotionForce() │
                                      │ Write state back to SHM      │
                                      └──────────────────────────────┘
```

## Key Design Decisions

| Decision | Rationale |
|---|---|
| Mutex-protected shared memory | Minimal contention at 1 kHz vs 100 Hz; simpler than lock-free for variable-size vectors |
| Pre-allocated RT buffers | Zero heap allocation in RT callback — uses `std::copy`, never `vector::operator=` |
| Split-lock RT callback | `robot_.states()` (slow IPC) called **outside** the mutex; lock held only for fast memory copies. Prevents Python contention from causing timeliness failures |
| Adaptive jump detection | Velocity-based thresholds scale with Python command frequency: `threshold = max_velocity × cmd_interval`. No manual tuning needed |
| Per-cycle velocity/acceleration clamping | Defense-in-depth: every 1ms cycle, position delta and velocity delta are clamped to Flexiv RDK safe defaults before sending to the robot |
| Linear interpolation for VLA streaming | Constant-velocity interpolation between sparse policy commands (e.g. 30 Hz). Avoids stop-and-go behavior of min-jerk at waypoint boundaries |
| `mlockall()` interception | Flexiv Scheduler calls `mlockall()` which pins all process memory; in a Python+PyTorch process this can be 10+ GB and triggers OOM. We intercept via linker `--wrap` and no-op it |
| Min-jerk trajectory in RT thread | Non-blocking `move_to_pose()` generates smooth 1 kHz commands without mode switching |
| 500ms command timeout | Holds last position if Python crashes or hangs |
| GIL release on blocking ops | `stop()`, `start_*_control()` release the GIL to avoid Python deadlock |
| Context managers | Ensures RT threads are stopped even on unhandled exceptions |

## Build & Install

Complete instructions from creating a fresh mamba environment to a working `import flexiv_rt`.

### Clone This Repository

Clone `libpyflexiv` together with the pinned `flexiv_rdk` submodule:

```bash
git clone --recurse-submodules https://github.com/XenseRobotics-AI/libpyflexiv.git
cd libpyflexiv
```

If you already cloned the repo without submodules, run:

```bash
git submodule update --init --recursive
```

### Prerequisites

- Ubuntu 22.04 x86_64
- [Mamba](https://mamba.readthedocs.io/) (or conda)
- `sudo` access (runtime SCHED_FIFO only)
- Standard build tools: `sudo apt install build-essential cmake git`

### Step 0: System Dependencies

```bash
sudo apt install build-essential cmake git
```

No ROS 2 required — all C++ dependencies (Eigen, spdlog, Fast-DDS, Fast-CDR, RBDyn, etc.) are built from source by the provided script in Step 2.

### Step 1: Create Mamba Environment

Create the environment first — all subsequent steps run inside it:

```bash
mamba create -n lerobot-xense python=3.1x -y
mamba activate lerobot-xense

# Build dependency
mamba install pybind11 -y
```

> All remaining steps assume `lerobot-xense` is activated.

### Step 2: Install Flexiv RDK and All Dependencies

The official `flexiv_rdk` SDK is included as a git submodule pinned to the tested `v1.8` tag commit. Initialize the submodule after cloning this repo, then build and install all its C++ dependencies to `~/rdk_install`, and finally build and install the RDK itself.

```bash
cd /path/to/libpyflexiv

# Step 2a: Build and install all C++ dependencies from source
# (Eigen, spdlog, tinyxml2, yaml-cpp, foonathan_memory,
#  Fast-CDR, Fast-DDS, Boost, SpaceVecAlg, RBDyn)
cd flexiv_rdk/thirdparty
bash build_and_install_dependencies.sh ~/rdk_install $(nproc)

# Step 2b: Build and install flexiv_rdk itself
cd /path/to/libpyflexiv/flexiv_rdk
mkdir -p build && cd build
cmake .. \
  -DCMAKE_INSTALL_PREFIX=~/rdk_install \
  -DCMAKE_PREFIX_PATH=~/rdk_install
cmake --build . --target install --config Release -j$(nproc)
```

> **Note**: No `sudo` needed — everything installs to your home directory.
> The RDK static library (`libflexiv_rdk.x86_64-linux-gnu.a`) is automatically downloaded
> from GitHub during the cmake configure step.
> The submodule is intentionally checked out at the pinned `v1.8` commit, so a detached `HEAD` inside `flexiv_rdk/` is expected.

Verify:
- `ls ~/rdk_install/include/flexiv/rdk/robot.hpp` should exist
- `ls ~/rdk_install/lib/cmake/flexiv_rdk/flexiv_rdk-config.cmake` should exist

### Step 3: Build C++ Bindings

Point CMake at the Flexiv RDK install prefix and at your conda Python interpreter.

**Important**: Only pass `~/rdk_install` as `CMAKE_PREFIX_PATH` — do NOT include the conda env prefix. The conda env contains a different spdlog version that will cause header conflicts.

```bash
cd /path/to/libpyflexiv
mkdir -p build && cd build

cmake .. \
  -DCMAKE_PREFIX_PATH=~/rdk_install \
  -DPython3_EXECUTABLE=$(which python)

make -j$(nproc)
```

This produces `flexiv_rt/_flexiv_rt.cpython-310-x86_64-linux-gnu.so` in the project root.

> **Troubleshooting**: If you see spdlog errors like `is_convertible_to_basic_format_string` or `basic_runtime is not a member of fmt`, it means cmake found a wrong spdlog. Ensure `CMAKE_PREFIX_PATH` is set to `~/rdk_install` and does NOT include the conda env prefix.

> **Troubleshooting (`ImportError: undefined symbol ... fmt::v8::detail::error_handler::on_error`)**:
> This means the extension was built against one fmt/spdlog ABI but loaded with another.
> Check linkage:
> ```bash
> so=flexiv_rt/_flexiv_rt.cpython-310-x86_64-linux-gnu.so
> readelf -d "$so" | egrep 'NEEDED|RPATH|RUNPATH'
> ldd "$so" | egrep 'spdlog|fmt'
> nm -D "$so" | c++filt | grep 'fmt::v8::detail::error_handler::on_error'
> ```
> Rebuild from scratch with only `~/rdk_install` in `CMAKE_PREFIX_PATH`:
> ```bash
> rm -rf build flexiv_rt/_flexiv_rt*.so
> cmake -S . -B build \
>   -DCMAKE_PREFIX_PATH=~/rdk_install \
>   -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
>   -DPython3_EXECUTABLE=$(which python)
> cmake --build build -j$(nproc)
> ```

### Step 4: Install Python Package

Editable install — the compiled `.so` is already in place:

```bash
pip install -e /path/to/libpyflexiv
```

Verify:

```bash
python -c "import flexiv_rt; print('OK')"
```

### Running (requires `sudo` for SCHED_FIFO)

The Flexiv RDK Scheduler creates SCHED_FIFO threads at maximum priority, which requires root. Use `sudo -E` to preserve your conda environment:

```bash
sudo -E mamba run -n lerobot-xense python your_script.py

# Or with explicit PATH preservation:
sudo -E env PATH=$PATH \
  LD_LIBRARY_PATH=$LD_LIBRARY_PATH \
  python your_script.py
```

## Quick Start

### Joint Impedance Control

```python
from flexiv_rt import Robot, Mode

robot = Robot("Rizon4s-XXXXXX")
robot.Enable()

# Home the robot
robot.SwitchMode(Mode.NRT_PLAN_EXECUTION)
robot.ExecutePlan("PLAN-Home")
import time; time.sleep(5)
robot.Stop()

# Switch to RT joint impedance
robot.SwitchMode(Mode.RT_JOINT_IMPEDANCE)
robot.SetJointImpedance(
    K_q=[3000, 3000, 800, 800, 200, 200, 200],  # stiffness
    Z_q=[0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7],   # damping ratio
)

# Start RT control — spawns C++ SCHED_FIFO thread at 1 kHz
with robot.start_joint_impedance_control() as ctrl:
    for i in range(5000):  # 5 seconds at 1 kHz Python-side
        state = ctrl.get_state()          # read current joint state
        target = list(state.q)            # copy current positions
        target[3] += 0.05 * math.sin(i * 0.001 * 2 * math.pi * 0.3)
        ctrl.set_target_joints(target)    # write to shared memory
        time.sleep(0.001)
    # Context manager auto-calls ctrl.stop()
```

### Cartesian Motion+Force Control

```python
robot.SwitchMode(Mode.RT_CARTESIAN_MOTION_FORCE)
robot.SetForceControlAxis([False]*6)  # pure motion, no force control

with robot.start_cartesian_control() as ctrl:
    state = ctrl.get_state()
    init_pose = list(state.tcp_pose)  # [x, y, z, qw, qx, qy, qz]

    for i in range(10000):  # 10 seconds
        pose = init_pose.copy()
        pose[1] += 0.05 * math.sin(i * 0.001 * 2 * math.pi * 0.2)  # Y sine
        ctrl.set_target_pose(pose)
        time.sleep(0.001)
```

### Non-Blocking Trajectory (Reset to Home)

```python
with robot.start_cartesian_control() as ctrl:
    # ... teleop streaming ...

    # User presses reset button:
    ctrl.move_to_pose(home_pose, duration_sec=3.0)  # returns immediately

    # Keep reading observations while robot moves
    while ctrl.is_moving():
        state = ctrl.get_state()  # observations still flowing
        time.sleep(0.01)

    # Trajectory complete — resume streaming
    ctrl.set_target_pose(home_pose)
    # ... continue teleop ...
```

## API Reference

### `Robot`

The main entry point. Wraps `flexiv::rdk::Robot` with connection retry logic, context manager support, and safe shutdown ordering.

```python
Robot(serial_number: str,
      network_whitelist: list[str] = [],
      connect_retries: int = 3,
      retry_interval_sec: float = 1.0,
      verbose: bool = True,
      lite: bool = False)
```

**Connection & Lifecycle**

| Method | Description |
|---|---|
| `Enable()` | Enable the robot (must be called after connection) |
| `Stop()` | Stop all motion. Stops active RT controls first, then sends robot stop. Releases GIL |
| `close()` | Full cleanup: stop all RT controls, stop robot, mark as closed. Idempotent |
| `ClearFault(timeout: int = 5)` | Clear robot fault with timeout in seconds |
| `SwitchMode(mode: Mode)` | Switch robot operating mode |

**State Accessors**

| Method | Returns | Description |
|---|---|---|
| `connected()` | `bool` | Whether robot is connected |
| `operational()` | `bool` | Whether robot is in operational state |
| `fault()` | `bool` | Whether robot has a fault |
| `busy()` | `bool` | Whether robot is busy executing a plan/primitive |
| `stopped()` | `bool` | Whether robot is stopped |
| `estop_released()` | `bool` | Whether e-stop is released |
| `reduced()` | `bool` | Whether robot is in reduced mode |
| `recovery()` | `bool` | Whether robot is in recovery mode |
| `mode()` | `Mode` | Current operating mode |
| `states()` | `RobotStates` | Full robot state snapshot (returns by value) |
| `info()` | `RobotInfo` | Static robot information (DoF, limits, serial number) |

**Plan & Primitive Execution**

| Method | Description |
|---|---|
| `ExecutePlan(name: str)` | Execute a named plan (e.g. `"PLAN-Home"`) |
| `ExecutePrimitive(name: str, params: dict)` | Execute a primitive with parameters. Special: `"MoveJ"` auto-converts `list[7]` target to `JPos` |

**Impedance Configuration** (call before starting RT control)

| Method | Parameters | Description |
|---|---|---|
| `SetJointImpedance(K_q, Z_q)` | `list[7]`, `list[7]` | Joint stiffness and damping ratio |
| `SetCartesianImpedance(K_x, Z_x)` | `list[6]`, `list[6]` | Cartesian stiffness `[Kx,Ky,Kz,Krx,Kry,Krz]` and damping |
| `SetMaxContactWrench(wrench)` | `list[6]` | Max contact wrench before protective stop |
| `SetMaxContactTorque(torques)` | `list[7]` | Max contact torque per joint |
| `SetForceControlAxis(axes, max_vel)` | `list[6] (bool)`, `list[3]` | Enable/disable force control per axis. `max_vel` default `[1,1,1]` |
| `SetForceControlFrame(coord, T)` | `CoordType`, `list[7]` | Set force control reference frame. `T` is `[x,y,z,qw,qx,qy,qz]` |
| `SetNullSpacePosture(ref_pos)` | `list[7]` | Preferred null-space joint positions |

**RT Control Entry Points**

| Method | Returns | Description |
|---|---|---|
| `start_joint_impedance_control(inner_control_hz=1000, interpolate_cmds=True)` | `JointImpedanceControl` | Spawns RT thread for joint streaming. `inner_control_hz` sets how often new Python commands are consumed (1–1000). `interpolate_cmds=True` enables linear interpolation between commands. Releases GIL |
| `start_cartesian_control(inner_control_hz=1000, interpolate_cmds=True)` | `CartesianMotionForceControl` | Spawns RT thread for Cartesian streaming. Same parameters as above. Releases GIL |

Both return context managers and maintain `keep_alive` references to prevent premature GC of the `Robot` object.

---

### `JointImpedanceControl`

Context manager for real-time joint impedance streaming. Created by `robot.start_joint_impedance_control()`.

**Usage**: Always use as a context manager or explicitly call `stop()`.

| Method | Description |
|---|---|
| `set_target_joints(positions, velocities=[], accelerations=[])` | Set joint target. All `list[7]` of `float`. Velocity/acceleration default to zero. Writes to shared memory (mutex-protected) |
| `get_state()` → `JointState` | Read current joint state from shared memory |
| `trigger_estop()` | Trigger emergency stop (lockless `atomic<bool>`, checked at top of every RT cycle) |
| `stop()` | Stop the RT thread. Releases GIL. Idempotent |
| `is_running()` → `bool` | Whether the RT thread is active |

---

### `CartesianMotionForceControl`

Context manager for real-time Cartesian streaming with integrated trajectory support. Created by `robot.start_cartesian_control()`.

**Streaming API** (RT state: `STREAMING`)

| Method | Description |
|---|---|
| `set_target_pose(pose, wrench=[], velocity=[], acceleration=[])` | Set Cartesian target. `pose`: `list[7]` `[x,y,z,qw,qx,qy,qz]`. `wrench`: `list[6]`. Writes to shared memory |
| `get_state()` → `CartesianState` | Read current Cartesian state from shared memory |

**Trajectory API** (RT state transitions: `STREAMING` → `MOVING` → `STREAMING`)

| Method | Description |
|---|---|
| `move_to_pose(pose, duration_sec=3.0)` | Start a min-jerk trajectory to `pose` (7D). **Non-blocking**: returns immediately, RT thread interpolates at 1 kHz. Idempotent if already moving to same target |
| `move_to_pose_sync(pose, duration_sec=3.0, timeout_sec=0.0)` | **Blocking** version. Releases GIL, polls at 10ms. Returns `True` on completion, `False` on timeout. `timeout_sec=0` means no timeout |
| `is_moving()` → `bool` | Whether a trajectory is in progress |
| `cancel_move()` | Cancel active trajectory. RT thread holds at current interpolated position |

**Lifecycle**

| Method | Description |
|---|---|
| `trigger_estop()` | Emergency stop (lockless atomic) |
| `stop()` | Stop RT thread. Releases GIL. Idempotent |
| `is_running()` → `bool` | Whether RT thread is active |

**State Machine**:
- During `STREAMING`: `set_target_pose()` commands are executed each RT cycle
- Calling `move_to_pose()` transitions to `MOVING`: RT thread follows the min-jerk trajectory, `set_target_pose()` calls are ignored
- When trajectory completes (or `cancel_move()`): transitions back to `STREAMING` with end pose as the hold target

---

### `JointState`

Snapshot of joint-space state, returned by `JointImpedanceControl.get_state()`.

| Field | Type | Description |
|---|---|---|
| `q` | `list[7]` | Joint positions (rad) |
| `dq` | `list[7]` | Joint velocities (rad/s) |
| `tau` | `list[7]` | Joint torques (Nm) |
| `tau_ext` | `list[7]` | External torques (Nm) |
| `tcp_pose` | `list[7]` | TCP pose `[x,y,z,qw,qx,qy,qz]` (convenience, always available) |

---

### `CartesianState`

Snapshot of Cartesian-space state, returned by `CartesianMotionForceControl.get_state()`.

| Field | Type | Description |
|---|---|---|
| `tcp_pose` | `list[7]` | TCP pose `[x,y,z,qw,qx,qy,qz]` (m, quaternion) |
| `tcp_vel` | `list[6]` | TCP velocity `[vx,vy,vz,wx,wy,wz]` (m/s, rad/s) |
| `ext_wrench_in_tcp` | `list[6]` | External wrench in TCP frame `[fx,fy,fz,mx,my,mz]` (N, Nm) |
| `ext_wrench_in_world` | `list[6]` | External wrench in world frame |
| `ft_sensor_raw` | `list[6]` | Raw F/T sensor readings |
| `q` | `list[7]` | Joint positions (rad) |
| `tau_ext` | `list[7]` | External joint torques (Nm) |

---

### `Mode` (Enum)

| Value | Description |
|---|---|
| `IDLE` | Robot idle |
| `RT_JOINT_TORQUE` | Real-time joint torque streaming |
| `RT_JOINT_IMPEDANCE` | Real-time joint impedance streaming |
| `RT_CARTESIAN_MOTION_FORCE` | Real-time Cartesian motion+force streaming |
| `NRT_JOINT_IMPEDANCE` | Non-real-time joint impedance |
| `NRT_CARTESIAN_MOTION_FORCE` | Non-real-time Cartesian motion+force |
| `NRT_PLAN_EXECUTION` | Plan execution mode |
| `NRT_PRIMITIVE_EXECUTION` | Primitive execution mode |
| `UNKNOWN` | Unknown mode |

### `CoordType` (Enum)

| Value | Description |
|---|---|
| `WORLD` | World (base) coordinate frame |
| `TCP` | Tool center point frame |

### `RobotInfo`

| Field | Type | Description |
|---|---|---|
| `serial_num` | `str` | Robot serial number |
| `model_name` | `str` | Robot model (e.g. "Rizon4s") |
| `DoF` | `int` | Degrees of freedom (7) |
| `K_q_nom` | `list[7]` | Nominal joint stiffness |
| `K_x_nom` | `list[6]` | Nominal Cartesian stiffness |
| `q_min` / `q_max` | `list[7]` | Joint position limits (rad) |
| `dq_max` | `list[7]` | Joint velocity limits (rad/s) |
| `tau_max` | `list[7]` | Joint torque limits (Nm) |

### `RobotStates`

Full robot state snapshot returned by `robot.states()`.

| Field | Type | Description |
|---|---|---|
| `q` | `list[7]` | Current joint positions (rad) |
| `theta` | `list[7]` | Motor-side joint positions (rad) |
| `dq` | `list[7]` | Joint velocities (rad/s) |
| `dtheta` | `list[7]` | Motor-side joint velocities (rad/s) |
| `tau` | `list[7]` | Joint torques (Nm) |
| `tau_des` | `list[7]` | Desired joint torques (Nm) |
| `tau_dot` | `list[7]` | Joint torque derivatives (Nm/s) |
| `tau_ext` | `list[7]` | External joint torques (Nm) |
| `tcp_pose` | `list[7]` | TCP pose `[x,y,z,qw,qx,qy,qz]` |
| `tcp_vel` | `list[6]` | TCP velocity `[vx,vy,vz,wx,wy,wz]` |
| `flange_pose` | `list[7]` | Flange pose |
| `ft_sensor_raw` | `list[6]` | Raw F/T sensor readings |
| `ext_wrench_in_tcp` | `list[6]` | External wrench in TCP frame |
| `ext_wrench_in_world` | `list[6]` | External wrench in world frame |

## Safety Features

The RT callback implements multiple safety layers, evaluated **every 1ms cycle**:

1. **Emergency Stop** — `atomic<bool>`, lockless check at the top of each cycle. Holds last sent command. Triggered by `trigger_estop()` or automatically on robot fault.

2. **NaN/Inf Detection** — Every command field is checked with `std::isfinite()`. Any NaN or Inf triggers an immediate e-stop.

3. **Adaptive Jump Detection** — Velocity-based thresholds that automatically adapt to the Python command frequency. The Python-side `set_target_pose()`/`set_target_joints()` measures the time interval between consecutive calls and stores it in shared memory. The RT thread computes per-step thresholds as:

   ```
   threshold = max_velocity × cmd_interval
   ```

   | Domain | Max Velocity | @100 Hz (10ms) | @30 Hz (33ms) | @10 Hz (100ms) |
   |---|---|---|---|---|
   | Joint position | 10 rad/s | 0.10 rad | 0.33 rad | 1.00 rad |
   | Cartesian position | 10 m/s | 100 mm | 330 mm | 1000 mm |
   | Cartesian rotation | 50 rad/s | 0.50 rad | 1.65 rad | 5.00 rad |

   The command interval is clamped to [1ms, 100ms] to prevent extreme thresholds. Before the second command arrives, the default interval (10ms / 100 Hz) is used. If a jump is detected, the RT thread clamps to the last sent position with zero velocity/acceleration.

4. **Per-Cycle Velocity/Acceleration Clamping** — Every 1ms cycle, **before** sending any command to the robot SDK, position deltas and velocity deltas are clamped to safe limits. This provides defense-in-depth even if interpolation or upstream code produces unexpected values.

   **Cartesian clamping** (`ClampCartesianPose` + `ClampCartesianVelocity` in `rt_common.hpp`):

   | Limit | Value | Per-Cycle Delta | Source |
   |---|---|---|---|
   | Linear velocity | 0.5 m/s | 0.5 mm/cycle | `SendCartesianMotionForce` default |
   | Angular velocity | 1.0 rad/s | 0.001 rad/cycle | `SendCartesianMotionForce` default |
   | Linear acceleration | 2.0 m/s² | 0.002 m/s per cycle | `SendCartesianMotionForce` default |
   | Angular acceleration | 5.0 rad/s² | 0.005 rad/s per cycle | `SendCartesianMotionForce` default |

   Position is clamped by Euclidean distance (preserving direction); rotation is clamped by angular distance using SLERP. Velocity feedforward is clamped by magnitude; acceleration is clamped per-axis.

   **Joint clamping** (`ClampJointPosition` + `ClampJointVelocity` in `rt_common.hpp`):

   | Limit | Value | Per-Cycle Delta | Source |
   |---|---|---|---|
   | Joint velocity | 2.0 rad/s | 0.002 rad/cycle | Flexiv RDK examples |
   | Joint acceleration | 3.0 rad/s² | 0.003 rad/s per cycle | Flexiv RDK examples |

   Joint position and velocity are clamped per-joint independently.

5. **Command Timeout** — If no new command arrives within **500ms**, the RT thread holds the last sent position with zero velocity/acceleration. Prevents runaway if Python crashes.

6. **Context Manager Cleanup** — `with robot.start_*_control() as ctrl:` guarantees `ctrl.stop()` is called even on exceptions, preventing orphaned RT threads.

7. **Idempotent Stop** — `stop()` uses `atomic<bool>` to ensure it's safe to call multiple times from multiple threads.

## Streaming Interpolation for VLA Policies

When using Vision-Language-Action (VLA) models like OpenPI, the policy outputs actions at a low frequency (e.g. 30 Hz) as discrete samples of a continuous trajectory. Without interpolation, the RT thread would hold the same position between commands and then snap to the next target, causing jerky motion.

The `inner_control_hz` and `interpolate_cmds` parameters solve this:

```python
# 30Hz VLA policy → smooth 1kHz motion via linear interpolation
with robot.start_cartesian_control(inner_control_hz=30, interpolate_cmds=True) as ctrl:
    for action in policy_actions:
        ctrl.set_target_pose(action)
        time.sleep(1/30)

# Same for joint control
with robot.start_joint_impedance_control(inner_control_hz=30, interpolate_cmds=True) as ctrl:
    for action in policy_actions:
        ctrl.set_target_joints(action)
        time.sleep(1/30)
```

### How It Works

The RT thread always runs at 1 kHz. The `inner_control_hz` parameter adds a **frequency decimator**: `decimation = round(1000 / inner_control_hz)`. The RT thread only reads a new Python command every `decimation` cycles.

When `interpolate_cmds=True`, each new command triggers **constant-velocity linear interpolation** from the current position to the target over one command period:

```
Position: pos(t) = start + (end - start) × t/T       (constant velocity)
Rotation: SLERP(q_start, q_end, t/T)                  (constant angular velocity)
Velocity feedforward: v = (end - start) / T            (constant)
```

This produces a **flat velocity profile** between waypoints — no deceleration at boundaries:

```
Linear interpolation (what we use):         Min-jerk (what we DON'T use):
velocity                                    velocity
  ^                                           ^
  |  ________  ________  ________             |   ╱╲    ╱╲    ╱╲
  | |        ||        ||        |            |  ╱  ╲  ╱  ╲  ╱  ╲
  +------------------------------→ time       +------------------------------→ time
    A        B         C        D               A     B     C     D
```

### Why Linear, Not Min-Jerk

Min-jerk (5th-order polynomial) ensures zero velocity at start and end of each segment. This is ideal for explicit trajectory moves (`move_to_pose()`), but **wrong for streaming VLA commands** where actions are continuous samples:

- Min-jerk decelerates to zero at every 33ms boundary → **stop-and-go sawtooth** velocity
- Average speed drops to ~60% of expected
- 30 Hz periodic acceleration/deceleration causes visible vibration

Linear interpolation treats each action as a waypoint to pass through at constant speed, matching the VLA policy's intent.

### Decimation and `robot_.states()` Optimization

The `robot_.states()` IPC call costs 0.2–0.8 ms per invocation. When using decimation (e.g. `inner_control_hz=30` → `decimation=33`), `robot_.states()` is only called on decimation boundary cycles, keeping non-boundary cycles extremely short. State staleness is at most `decimation` ms, which is invisible to the Python control loop.

## Min-Jerk Trajectory

The `move_to_pose()` API uses a minimum-jerk (5th-order polynomial) trajectory generator that runs entirely in the RT thread:

**Position**: `s(t) = 10t^3 - 15t^4 + 6t^5` (normalized time `t = elapsed / duration`)

**Rotation**: Quaternion SLERP with short-path selection (dot product sign correction)

**Velocity feed-forward**: Analytical derivative `ds/dt = (30t^2 - 60t^3 + 30t^4) / duration`

Properties:
- Zero velocity and acceleration at start and end
- Smooth, continuous commands at 1 kHz
- Zero heap allocation (all buffers pre-allocated)
- Safe to call from any thread (mutex-protected request handoff to RT thread)

## Project Structure

```
libpyflexiv/
├── CMakeLists.txt                  # Top-level build (pybind11 module)
├── pyproject.toml                  # Python package metadata
├── setup.py                        # pip install support
├── flexiv_rdk/                     # Flexiv RDK v1.8 SDK submodule
├── include/realtime_control/       # C++ RT control headers
│   ├── rt_common.hpp               #   Safety constants, checks, per-cycle clamping
│   ├── shared_memory.hpp           #   SPSC ring buffer (alternative design)
│   ├── joint_state.hpp             #   JointState struct
│   ├── cartesian_state.hpp         #   CartesianState struct
│   ├── joint_impedance_control.hpp #   Joint RT controller declaration
│   ├── cartesian_control.hpp       #   Cartesian RT controller declaration
│   ├── trajectory.hpp              #   MinJerkTrajectory + LinearTrajectory
│   └── logging.hpp                 #   Logger setup
├── src/                            # C++ implementation
│   ├── flexiv_rt.cpp               #   Pybind11 module (Robot, controls, enums)
│   ├── joint_impedance_control.cpp #   Joint RT callback + linear interpolation
│   └── cartesian_control.cpp       #   Cartesian RT callback + mlockall wrap
├── flexiv_rt/                      # Python package
│   ├── __init__.py                 #   Re-exports from compiled .so
│   └── _flexiv_rt.*.so             #   Compiled pybind11 module
├── examples/
│   ├── joint_impedance_example.py  #   Joint sine-sweep demo
│   ├── cartesian_motion_example.py #   Cartesian Y-axis sine demo
│   └── rt_reset_example.py         #   Non-blocking trajectory reset demo
└── tests/
    ├── cpp/
    │   ├── CMakeLists.txt          #   GTest build config
    │   ├── test_rt_safety.cpp      #   Safety functions, buffer, clamping tests
    │   └── test_trajectory.cpp     #   LinearTrajectory + MinJerkTrajectory tests
    └── python/
        ├── conftest.py             #   Pytest fixtures (robot connection)
        ├── test_rt_integration.py  #   Integration tests (requires real robot)
        └── test_interpolation_sim.py # Interpolation simulation (no robot needed)
```

## Tests

Three test suites, covering different layers without requiring a real robot (except integration tests).

### C++ Unit Tests (GTest, no robot required)

Build and run:

```bash
cd tests/cpp
mkdir -p build && cd build
cmake .. && make -j$(nproc)
./test_rt_safety      # 45 tests: safety checks, buffer, per-cycle clamping
./test_trajectory     # 14 tests: LinearTrajectory, MinJerkTrajectory, VLA streaming sim
```

**test_rt_safety.cpp** — 45 tests across 10 suites:

| Suite | Tests | What it covers |
|---|---|---|
| `QuatAngularDist` | 5 | Quaternion angular distance (identity, small/large rotation, antipodal) |
| `CheckFinite` | 6 | NaN/Inf detection on vectors and arrays |
| `CheckJointJump` | 4 | Joint position jump detection (threshold, size mismatch, edge cases) |
| `CheckCartesianJump` | 4 | Position + rotation jump detection |
| `CacheLineAlign` | 1 | Cache line size constant verification |
| `RealTimeBuffer` | 11 | SPSC ring buffer: FIFO, wrap-around, full rejection, concurrent stress (100k items) |
| `ClampCartesianPose` | 5 | Position Euclidean clamping, rotation angular clamping via SLERP, diagonal direction preservation |
| `ClampCartesianVelocity` | 4 | Linear/angular velocity magnitude clamping, linear acceleration clamping |
| `ClampJointPosition` | 3 | Per-joint velocity limiting (positive/negative delta) |
| `ClampJointVelocity` | 2 | Per-joint acceleration limiting |

**test_trajectory.cpp** — 14 tests across 3 suites:

| Suite | Tests | What it covers |
|---|---|---|
| `LinearTrajectory` | 11 | Constant velocity, linear position, SLERP constant angular velocity, halfway angle accuracy, combined translation+rotation, monotonic progress, zero motion, cancel, min duration, inactive before init, **VLA streaming simulation (no zero-velocity dips)** |
| `TrajectoryComparison` | 2 | Linear has constant velocity vs MinJerk varies; both reach same end pose |
| `MinJerkTrajectory` | 1 | Confirms stop-and-go: first-step velocity near zero at every segment boundary |

### Python Simulation Tests (no robot required)

```bash
# Run assertions only
python tests/python/test_interpolation_sim.py

# Also generate comparison plots
python tests/python/test_interpolation_sim.py --plot
```

**test_interpolation_sim.py** — 7 tests:

| Test | What it validates |
|---|---|
| Cartesian linear: constant velocity | Velocity deviation < 1e-10 across all 1kHz cycles |
| Cartesian min-jerk: stop-and-go | Confirms first-step velocity near zero at each segment boundary |
| Joint linear: constant velocity | All 7 joints have constant velocity, final position exact |
| Joint min-jerk: velocity varies | Velocity range within each segment > 0.01 (not constant) |
| Rotation SLERP: constant angular velocity | Position stays constant during pure rotation |
| End position accuracy | Final position error < 1e-10 after 50 random waypoints |
| Multiple policy frequencies | Linear interpolation correct at 10/20/30/50/100/200/500 Hz |

The `--plot` flag generates `interpolation_comparison.png` showing linear vs min-jerk velocity profiles side-by-side for both Cartesian and joint control.

### Integration Tests (requires real robot)

```bash
sudo -E pytest tests/python/ --robot-sn Rizon4s-XXXXXX -v -s --timeout=120
```

Requires a physical Flexiv robot connected on the network. See `tests/python/conftest.py` for fixtures and `test_rt_integration.py` for the full test suite (23 tests covering connection, mode switching, joint/Cartesian control, safety, and lerobot compatibility).

## License

See `flexiv_rdk/LICENSE` for the Flexiv RDK license.
