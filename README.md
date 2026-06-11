# xarm_lm

ROS2 package implementing **gaze stabilization** for the xArm6 robot using the **Levenberg-Marquardt (LM) algorithm** for inverse kinematics. The end-effector is kept fixed in the world frame while the robot base undergoes arbitrary disturbances — analogous to a chicken's head remaining still as its body moves.

<video src="media/demo.webm" controls width="100%"></video>

---

## How It Works

### The Problem

When a robot arm is mounted on a moving platform (drone, mobile base, ship deck), the end-effector drifts in world space as the base moves — even if the joint angles stay constant. Gaze stabilization compensates for this by continuously commanding joint motions that cancel out the base disturbance.

### The Control Loop

The system is **purely reactive** (closed-loop). It has no model of how the base moves — it simply observes the error and corrects it:

```
base moves → EE drifts in world frame (TF changes)
           → gaze_stabilizer reads pose error from TF
           → P controller: v_cmd = Kp × error  (Cartesian velocity)
           → rotate v_cmd into link_base frame
           → publish TwistStamped to MoveIt Servo
           → LM-IK: dq = Jᵀ(JJᵀ + λ²I)⁻¹ · v_cmd
           → joints move → error shrinks → repeat at 50 Hz
```

### The Levenberg-Marquardt IK Step

At each control tick, MoveIt Servo solves the velocity-level IK using **damped least-squares**, which is the Levenberg-Marquardt method applied to the Jacobian:

```
dq = Jᵀ (J Jᵀ + λ²I)⁻¹ · v_cmd
```

| Symbol | Meaning |
|--------|---------|
| `J`    | 6×6 Jacobian matrix of the xArm6 at the current configuration |
| `dq`   | Joint velocity vector (6 DOF) to command |
| `v_cmd`| Desired end-effector Cartesian velocity (3 linear + 3 angular) |
| `λ`    | Damping factor — the "LM" regularization term |

**Why this is better than plain Jacobian inversion:**  
The naive solution `dq = J⁻¹ · v` blows up when the arm is near a singularity (J becomes rank-deficient). The `λ²I` term keeps the solution bounded. When the arm is far from singularities `λ` is small and the solution is accurate. When near a singularity `λ` grows, trading accuracy for stability — the joints slow down gracefully rather than flying to extreme values.

### Base Disturbance

The base is simulated by publishing a time-varying `world → link_base` TF transform that overrides the static transform from the simulation. The motion pattern combines:

- **XY plane:** Lemniscate of Bernoulli (infinity / figure-8 shape)
  ```
  x(t) = A · cos(θ) / (1 + sin²(θ))
  y(t) = A · sin(θ)·cos(θ) / (1 + sin²(θ))     θ = ωt
  ```
- **Z axis:** Sinusoidal oscillation
  ```
  z(t) = A_z · sin(ω_z · t)
  ```

---

## Package Structure

```
xarm_lm/
├── xarm_lm/
│   ├── gaze_stabilizer.py   # Main control node
│   ├── base_disturbance.py  # Simulated base motion
│   └── initial_pose.py      # Startup utility
└── launch/
    └── gaze_stabilizer.launch.py
```

---

## Dependencies

- `xarm_ros2` (xArm description, controllers, MoveIt config, MoveIt Servo)
- `moveit_servo` (ROS2 Humble)
- `rclpy`, `tf2_ros`, `geometry_msgs`, `std_msgs`, `std_srvs`, `trajectory_msgs`
- Python: `numpy`, `scipy`

---

## Usage

### Launch

```bash
ros2 launch xarm_lm gaze_stabilizer.launch.py
```

With custom disturbance parameters:

```bash
ros2 launch xarm_lm gaze_stabilizer.launch.py \
  amplitude:=0.10  z_amplitude:=0.05 \
  period:=8.0      z_period:=4.0
```

### Workflow

1. **RViz opens** with the MoveIt motion planning panel. The arm automatically moves to a non-singular ready pose.

2. **Set the desired EE pose** by dragging the interactive orange marker in RViz to the position you want to hold, then click **Plan & Execute**.

3. **Start stabilization** — lock the current EE pose and enable the base disturbance:
   ```bash
   ros2 topic pub --once /gaze_stabilizer/start std_msgs/msg/Bool "{data: true}"
   ```
   The base begins moving in the lemniscate + Z pattern. The arm compensates in real time.

4. **Stop stabilization:**
   ```bash
   ros2 topic pub --once /gaze_stabilizer/start std_msgs/msg/Bool "{data: false}"
   ```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `amplitude` | `0.08` | XY disturbance half-width (m) |
| `z_amplitude` | `0.04` | Z disturbance half-height (m) |
| `period` | `8.0` | XY lemniscate period (s) |
| `z_period` | `4.0` | Z oscillation period (s) |
| `kp_linear` | `16.0` | Proportional gain for position error |
| `kp_angular` | `12.0` | Proportional gain for orientation error |

---

## Nodes

### `gaze_stabilizer`

The main control node. Implements the outer proportional control loop that feeds velocity commands to MoveIt Servo.

**Behavior:**
- On startup, automatically calls `/servo_server/start_servo` to bring the servo out of its paused state.
- Waits idle until it receives a `True` on `~/start`.
- On start: reads the current EE pose from TF (`world → link_eef`) and latches it as the desired pose. Also sends `True` to `/base_disturbance/enable` to start the disturbance simultaneously.
- Each tick at 50 Hz: computes pose error, applies proportional gains, rotates the velocity into `link_base` frame, and publishes a `TwistStamped` to `/servo_server/delta_twist_cmds`.
- On stop (`False` on `~/start`): clears the latch and disables the disturbance.

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `kp_linear` | `16.0` | Position error gain |
| `kp_angular` | `12.0` | Orientation error gain |
| `max_linear_vel` | `0.5` | Velocity clamp (m/s) |
| `max_angular_vel` | `2.0` | Angular velocity clamp (rad/s) |
| `world_frame` | `world` | World/fixed frame name |
| `ee_frame` | `link_eef` | End-effector frame name |
| `base_frame` | `link_base` | Robot base frame name |
| `control_rate` | `50.0` | Control loop frequency (Hz) |

**Topics:**

| Topic | Type | Direction |
|-------|------|-----------|
| `~/start` | `std_msgs/Bool` | Subscribe |
| `/servo_server/delta_twist_cmds` | `geometry_msgs/TwistStamped` | Publish |
| `/base_disturbance/enable` | `std_msgs/Bool` | Publish |

---

### `base_disturbance`

Simulates a moving robot base by publishing a time-varying `world → link_base` TF at 50 Hz. Starts disabled and is activated by `gaze_stabilizer` when stabilization begins.

The dynamic TF on `/tf` takes precedence over the static `world → link_base` published by the MoveIt fake simulation, so no upstream files are modified.

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `amplitude` | `0.08` | XY lemniscate half-width (m) |
| `z_amplitude` | `0.04` | Z oscillation half-height (m) |
| `period` | `8.0` | XY loop period (s) |
| `z_period` | `4.0` | Z oscillation period (s) |
| `publish_rate` | `50.0` | TF publish rate (Hz) |
| `world_frame` | `world` | Parent frame |
| `base_frame` | `link_base` | Child frame |

**Topics:**

| Topic | Type | Direction |
|-------|------|-----------|
| `~/enable` | `std_msgs/Bool` | Subscribe |

---

### `initial_pose`

One-shot utility node. Waits 1.5 s for the joint trajectory controller to finish activating, then sends the arm to a non-singular ready pose `[0, -1.0, 0, 0, 1.0, 0]` (rad) and keeps spinning so the process stays alive in the launch graph.

This pose was chosen to keep all joints well within their limits and away from kinematic singularities, giving the LM solver a well-conditioned Jacobian across the disturbance range.

**xArm6 joint limits (for reference):**

| Joint | Lower (rad) | Upper (rad) |
|-------|-------------|-------------|
| joint1 | -3.11 | +3.11 |
| joint2 | -2.06 | +2.09 |
| joint3 | -3.11 | **+0.19** |
| joint4 | -3.11 | +3.11 |
| joint5 | -1.69 | +3.11 |
| joint6 | -3.11 | +3.11 |

> Note: `joint3` has a very small upper limit (+0.19 rad ≈ 11°). Any ready pose must respect this.

---

## Notes

- **Lag vs. accuracy:** The controller is proportional (P-only). There is inherent lag between base motion and arm compensation because the arm waits to observe the error before correcting. A feedforward term — using the measured base velocity directly in `v_cmd` — would reduce this lag.
- **Singularities:** The LM damping (`λ`) handles singular configurations gracefully. The `hard_stop_singularity_threshold` is set high (200) to avoid emergency stops during large disturbances.
- **Amplitude limits:** For large amplitudes, pick EE poses near the center of the workspace. Poses near full arm extension are closer to singularities and will show more lag.
