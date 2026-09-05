# Pioneer P3-DX Point-to-Point Trajectory Control (ROS / Gazebo)

A ROS control node that drives a simulated **Pioneer P3-DX** mobile robot through a three-point trajectory using **cubic-polynomial motion profiles**, validated in **Gazebo** and verified against recorded odometry data.

> University Robotics course project, University of Ioannina — Academic Year 2021–2022.
> Co-developed with a teammate as a two-person team project.

## Problem

Drive the robot from a start pose to an intermediate point, then to a final point with a specified final orientation — using only the maximum linear and angular speed limits given in the assignment (`u_max = 0.2 m/s`, `ω_max = 40°/s`):

| Pose | Position | Orientation |
|---|---|---|
| `q₀` (start) | (0, 0) m | 0 rad |
| `q_v` (intermediate) | (24, 12) m | — |
| `q_f` (final) | (24, −12) m | 2.4145 rad |

The robot's motion is broken into five sequential phases:

1. **Rotate** in place to face the intermediate point.
2. **Translate** in a straight line to the intermediate point (`s₁ = √(24² + 12²) ≈ 26.83 m`).
3. **Rotate** in place to face the final point.
4. **Translate** in a straight line to the final point (`s₂ = 24 m`).
5. **Rotate** in place to the required final orientation (`2.4145 rad`).

## Approach: cubic polynomial trajectories

Rather than commanding a constant velocity (which would produce a discontinuous jerk at the start and end of each phase), each rotation and translation phase is generated as a **cubic polynomial** in time with zero velocity at both endpoints:

```
θ(t) = a₀ + a₁t + a₂t² + a₃t³
a₀ = θ₀,  a₁ = 0,
a₂ = 3(θf − θ₀) / tf²,
a₃ = −2(θf − θ₀) / tf³
```

Differentiating gives the corresponding smooth angular/linear velocity profile that is published to the robot at each control step. The same construction is used for the two straight-line translation phases, substituting distance `d` for angle `θ`.

Each phase duration `tf` was chosen as roughly 50% longer than the theoretical minimum time implied by the velocity limits (`t_min = distance / u_max` or `angle / ω_max`), so that the peak of the resulting smooth velocity profile stays safely under the platform's speed limits. This was verified analytically in MATLAB (see below) before being deployed to the ROS node.

## Repository Contents

| File | Description |
|---|---|
| `Project.pdf` | Assignment specification, derivation of the cubic-polynomial coefficients, and the final report with theoretical vs. measured trajectory plots |
| `final.py` | Main ROS node — publishes the full 5-phase trajectory as `Twist` commands to the robot |
| `dumm.py` | Standalone script used to check the cubic-polynomial coefficients numerically before wiring them into the ROS node |
| `myfirstnode.py` | Minimal ROS "hello world" node used to confirm the workspace and `rospy` setup |
| `rotate.py` | Small test publisher that alternates rotation direction every 10 s, used to sanity-check the `cmd_vel` topic and sign conventions |
| `move.py` | Small test publisher that sends a constant linear + angular velocity, used as an early connectivity check with the robot/simulator |
| `theta.m` | MATLAB script plotting the theoretical angular position `θ(t)` for each of the three rotation phases |
| `theta_dot.m` | MATLAB script plotting the theoretical angular velocity `θ̇(t)` for each rotation phase |
| `velocity.m` | MATLAB script plotting the theoretical linear distance and linear velocity profiles for each translation phase |

## Running It

```bash
# Terminal 1
roscore

# Terminal 2
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch p3dx_gazebo p3dx_empty_world.launch

# Terminal 3
rosrun p3dx_gazebo final.py
```

`final.py` is expected at `~/catkin_ws/src/p3dx/p3dx_gazebo/scripts/`.

## Verification

The robot's actual motion was recorded during simulation with `rosbag record`, exported to `.csv`, and plotted against the theoretical velocity profiles derived above. The measured trajectory reached an intermediate point of **(25.10, 9.49) m** and a final point of **(22.96, −14.41) m**, closely matching the theoretical targets of (24, 12) and (24, −12) — the small deviation is attributable to open-loop dead-reckoning drift, since the controller is a pure feed-forward velocity profile with no closed-loop position feedback. Measured peak linear velocities (≈0.16–0.18 m/s) stayed within the 0.2 m/s platform limit, and measured peak angular velocities stayed within the 0.698 rad/s (40°/s) limit, matching the theoretical design.

## Notes on the test scripts

`myfirstnode.py`, `rotate.py` and `move.py` were early scaffolding scripts used to validate the ROS/Gazebo setup and the `/pioneer/RosAria/cmd_vel` topic before writing the full trajectory controller in `final.py` — they are kept in the repository as a record of the development process rather than as separate deliverables.

## Author

**Stefanos Fotopoulos** (co-developed with a teammate)
[LinkedIn](https://www.linkedin.com/in/stefanos-fotopoulos-95192a290/) · [GitHub](https://github.com/stef-fot)

## License

MIT — see [`License`](./License).
