# Mechanical Stability (ZMP) in `perceptive_mpc`

This note explains how the **Zero Moment Point (ZMP)** and the mechanical stability soft constraint are implemented and how to use them at runtime.

## Closed-form ZMP used in the code
In the base frame, with surface normal $n = [0, 0, 1]^\top$ (vertical to ground), center of mass $r_{\rm{COM}}$, end-effector position $r_{EE}$, gravity $f_g$ and commanded interaction wrench $\mathcal{F}_{EE} = (f_{EE},\ \tau_{EE})$, the planar moment balance at the ZMP is

$$
0 = n \times \Big[(r_{\rm{COM}} - r_{\rm{ZMP}}) \times f_g - (r_{EE} - r_{\rm{ZMP}}) \times f_{EE} - \tau_{EE}\Big].
$$

Solving for the ZMP location gives the expression implemented in code:

$$
r_{\rm{ZMP}} = \frac{n \times (\ r_{\rm{COM}} \times f_g - r_{EE} \times f_{EE} - \tau_{EE}\ )}{n \cdot (\ f_g - f_{EE}\ )}.
$$

We constrain the ZMP to lie **inside** a support circle of radius $r_{sc}$ centered at the support polygon origin:

$$
g_{\rm{ZMP}}(x) = r_{sc}^2 - \Vert r_{\rm{ZMP}}\Vert_2^2 \ge 0.
$$

## Implemenations in the code
- **ZMP computation:** [`src/kinematics/KinematicsInterface.cpp:147-157`](../src/kinematics/KinematicsInterface.cpp)
  - Computes $r_{\rm{COM}}$ via `getCOMBaseFrame`.
  - Computes $r_{\rm{ZMP}}$ via `getXMPBaseFrame`.
  - Transforms the desired end-effector wrench from EE frame to base frame.
  - Uses the closed-form equation above to return $r_{\rm{ZMP}}$ in the base frame.

- **Soft constraint evaluation:** `StabilitySoftConstraint::intermediateCostFunction` ([`src/costs/StabilitySoftConstraint.cpp:38`](../src/costs/StabilitySoftConstraint.cpp)).
  - Interpolates the reference pose+wrench from the MPC trajectory.
  - Calls `getZMPBaseFrame` and evaluates $g_{\rm{ZMP}} = r_{sc}^2 - (x_{\rm{ZMP}}^2 + y_{\rm{ZMP}}^2)$.
  - Feeds that into a relaxed barrier (configurable $\mu,\delta$).

- **Wiring into MPC:** `PerceptiveMpcInterface` ([`example/PerceptiveMpcInterface.cpp:154`](../example/PerceptiveMpcInterface.cpp)).
  - Loads the flag and parameters from the task file (see below).
  - If enabled, instantiates `StabilitySoftConstraint` with the chosen support circle radius.

- **Runtime visualization:** [`example/KinematicSimulation.cpp:465`](../example/KinematicSimulation.cpp)
  - `KinematicSimulation::publishZmp` publishes COM (`/perceptive_mpc/com`) and ZMP (`/perceptive_mpc/zmp`) as `geometry_msgs/PointStamped` w.r.t. `base_link`.

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
The MPC will then keep $r_{\rm{ZMP}}$ **inside** the support circle while following the pose/wrench trajectory.

## API touch points
- Compute ZMP directly (double): `kinematicsInterface.getZMPBaseFrame(state, desiredPose, desiredWrench)` ([`src/kinematics/KinematicsInterface.cpp:157`](../src/kinematics/KinematicsInterface.cpp)).
- Use as a soft constraint in MPC: enable the block above; the constraint is added automatically during `PerceptiveMpcInterface` setup.
- Observe at runtime: subscribe to `/perceptive_mpc/zmp` and `/perceptive_mpc/com` in `base_link`.
