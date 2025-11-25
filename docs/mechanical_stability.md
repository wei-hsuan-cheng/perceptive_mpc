# Mechanical Stability (ZMP) in `perceptive_mpc`

This note explains how the **Zero Moment Point (ZMP)** and the mechanical stability soft constraint are implemented and how to use them at runtime.

## Closed-form ZMP used in the code
In the base frame, with surface normal $n = [0, 0, 1]^\top$ (vertical to ground), center of mass $r_{\text{COM}}$, end-effector position $r_{EE}$, gravity $f_g$ and commanded interaction wrench $\mathcal{F}_{EE} = (f_{EE},\ \tau_{EE})$, the planar moment balance at the ZMP is
$$
0 = n \times \Big[(r_{\text{COM}} - r_{\text{ZMP}}) \times f_g - (r_{EE} - r_{\text{ZMP}}) \times f_{EE} - \tau_{EE}\Big].
$$

Solving for the ZMP location gives the expression implemented in code:
$$
r_{\text{ZMP}} = \frac{n \times (\ r_{\text{COM}} \times f_g - r_{EE} \times f_{EE} - \tau_{EE}\ )}{n \cdot (\ f_g - f_{EE}\ )}.
$$

We constrain the ZMP to lie **inside** a support circle of radius $r_{sc}$ centered at the support polygon origin:
$$
g_{\text{ZMP}}(x) = r_{sc}^2 - \|r_{\text{ZMP}}\|_2^2 \ge 0.
$$

## Implemenations in the code
- **ZMP computation:** `KinematicsInterface::getZMPBaseFrame` (`src/kinematics/KinematicsInterface.cpp:157`).
  - Computes $r_{\text{COM}}$ via `getCOMBaseFrame`.
  - Transforms the desired end-effector wrench from EE frame to base frame.
  - Uses the closed-form equation above to return $r_{\text{ZMP}}$ in the base frame.
- **Soft constraint evaluation:** `StabilitySoftConstraint::intermediateCostFunction` (`src/costs/StabilitySoftConstraint.cpp:20`).
  - Interpolates the reference pose+wrench from the MPC trajectory.
  - Calls `getZMPBaseFrame` and evaluates $g_{\text{ZMP}} = r_{sc}^2 - (x_{\text{ZMP}}^2 + y_{\text{ZMP}}^2)$.
  - Feeds that into a relaxed barrier (configurable $\mu,\delta$).
- **Wiring into MPC:** `PerceptiveMpcInterface` (`example/PerceptiveMpcInterface.cpp:149`).
  - Loads the flag and parameters from the task file (see below).
  - If enabled, instantiates `StabilitySoftConstraint` with the chosen support circle radius.
- **Runtime visualization:** `KinematicSimulation::publishZmp` (`example/KinematicSimulation.cpp:456`).
  - Publishes COM (`/perceptive_mpc/com`) and ZMP (`/perceptive_mpc/zmp`) as `geometry_msgs/PointStamped` in `base_link`.

## How to enable/tune it
Edit `config/task.info` under the `stability_soft_constraint` block:
```bash
stability_soft_constraint
{
    activate        1        ; turn the soft constraint on/off
    mu              5e-3     ; relaxed barrier slope
    delta           1e-3     ; relaxed barrier offset
    support_circle_radius 0.3  ; r_sc [m]
}
```
The MPC will then keep $r_{\text{ZMP}}$ inside the support circle while following the pose/wrench trajectory.

## API touch points
- Compute ZMP directly (double): `kinematicsInterface.getZMPBaseFrame(state, desiredPose, desiredWrench)` (`include/perceptive_mpc/kinematics/KinematicsInterface.hpp:62`).
- Use as a soft constraint in MPC: enable the block above; the constraint is added automatically during `PerceptiveMpcInterface` setup.
- Observe at runtime: subscribe to `/perceptive_mpc/zmp` and `/perceptive_mpc/com` in `base_link`.
