# NMPC PID Controller GDP

MATLAB closed-loop path-following framework for the Cranfield Kia Niro automated vehicle platform, developed for the MSc GDP project.

This repository implements a decoupled vehicle control architecture using:

- Nonlinear Model Predictive Control (NMPC) for lateral path tracking
- PID control for longitudinal speed tracking
- A vehicle plant model for closed-loop simulation
- Lookup-based longitudinal actuator modelling for throttle and brake behaviour

ROS2 node repository:
`https://github.com/YuxuanCAVE/vehicle_controller`

---

## Overview

The project simulates a closed-loop autonomous vehicle control system in MATLAB.

The control structure is separated into:

1. **Lateral control**
   - Tracks the reference path.
   - Uses NMPC based on a kinematic bicycle model.
   - Outputs the steering command.

2. **Longitudinal control**
   - Tracks the reference speed.
   - Uses a PID controller.
   - Outputs the desired longitudinal acceleration.

3. **Plant model**
   - Simulates the vehicle response to steering, throttle, and brake commands.
   - Includes lateral vehicle dynamics and longitudinal force-balance behaviour.
   - Allows controller performance to be tested before ROS2 implementation.

The current baseline setup is:

- lateral controller: `mpc_kinematic`
- longitudinal controller: `pid`
- lateral plant: selectable `dynamic` or `bicycle_linear`
- longitudinal actuator model: 1D lookup inversion plus 1D forward execution
- outputs: run directory with metrics, plots, and optional simulation-vs-bag comparison

The active default configuration is defined in:

```matlab
config/default_config.m
```

If this README and the code ever differ, trust the code.

---

## Current Default Setup

The current default controller and plant selection is:

```matlab
cfg.controller.lateral = "mpc_kinematic";
cfg.controller.longitudinal = "pid";
cfg.plant.lateral_model = "dynamic";   % "dynamic" | "bicycle_linear"
```

Important simulation defaults:

```matlab
cfg.sim.dt = 0.1;
cfg.sim.T_end = 150;

cfg.speed.mode = "constant";
cfg.speed.constant_value = 3.0;
cfg.speed.profile_file = fullfile( ...
    'data', 'reference_velocity', 'referencePath_Velocity_peak_velocity_3.mat');

cfg.vehicle.max_steer = deg2rad(20);
cfg.vehicle.max_steer_rate = deg2rad(15);
cfg.vehicle.delay.steer_s = 0.1;
cfg.vehicle.delay.longitudinal_s = 0.1;
```

Current NMPC baseline:

```matlab
cfg.mpc_kinematic.N = 16;
cfg.mpc_kinematic.Q = diag([2.2, 0.8]);
cfg.mpc_kinematic.R = 25;
cfg.mpc_kinematic.Rd = 15.0;
cfg.mpc_kinematic.kappa_ff_gain = 0.1;
cfg.mpc_kinematic.fallback_k_e_y = 0.9;
cfg.mpc_kinematic.fallback_k_e_psi = 1.4;
```

Current PID baseline:

```matlab
cfg.lon_pid.kp = 1.6;
cfg.lon_pid.ki = 0.0;
cfg.lon_pid.kd = 0.0;
```

---

## Control Architecture

The overall controller is divided into two main parts:

```text
Reference path  -> NMPC lateral controller -> Steering command
Reference speed -> PID longitudinal control -> Desired acceleration
```

The plant then converts these commands into vehicle motion:

```text
Steering command + longitudinal force
        -> vehicle plant model
        -> vehicle position, yaw, velocity, and tracking errors
```

---

## Lateral NMPC Controller

The lateral controller is responsible for keeping the vehicle close to the reference path.

The controller uses a kinematic bicycle model to predict the future motion of the vehicle over a finite prediction horizon.

The lateral state is generally represented as:

```matlab
x = [X, Y, psi]
```

where:

- `X` is the global x-position of the vehicle
- `Y` is the global y-position of the vehicle
- `psi` is the vehicle yaw angle

The lateral control input is:

```matlab
u = delta
```

where:

- `delta` is the front steering angle

The NMPC controller calculates the steering command by minimizing path-tracking error and steering effort over the prediction horizon.

The main NMPC output is:

```matlab
delta_cmd
```

This steering command is then passed through:

1. steering delay buffer
2. steering rate limiter
3. lateral plant model

---

## Longitudinal PID Controller

The longitudinal controller is responsible for tracking the reference vehicle speed.

The speed tracking error is:

```matlab
e_v = v_ref - v
```

where:

- `v_ref` is the reference speed
- `v` is the current vehicle speed

The PID controller calculates the desired longitudinal acceleration:

```matlab
a_des = kp * e_v + ki * integral(e_v) + kd * d(e_v)/dt
```

The PID function interface is:

```matlab
[a_des, lon] = PID_controller(v_ref, v, lon, dt)
```

The PID output is a desired net acceleration correction only.

Resistance compensation is not added inside the PID controller. Instead, resistance is added later by the longitudinal force-balance model:

```matlab
F_required = M * a_des + F_resist
```

This separation means:

- the controller decides the desired acceleration
- the plant decides the force required to achieve it

---

## Plant Model

The plant model simulates how the vehicle responds to the controller commands.

The plant includes:

- lateral vehicle response
- longitudinal force balance
- actuator delay
- steering rate limits
- throttle/brake lookup behaviour

### Lateral Plant

The lateral plant can be selected using:

```matlab
cfg.plant.lateral_model = "dynamic";
```

Available options:

| Model | Description |
|---|---|
| `dynamic` | Nonlinear tire-force model using `lateral_tire_model.m` |
| `bicycle_linear` | Linear bicycle tire-force model with force saturation |

The lateral execution chain is:

```text
reference path
  -> lateral controller
  -> steering delay buffer
  -> steering rate limit
  -> lateral plant
  -> coupled_bicycle_dynamics
```

### Longitudinal Plant

The longitudinal plant converts the desired acceleration into a required force.

The main force-balance equation is:

```matlab
F_required = M * a_des + F_resist
```

where:

- `F_required` is the required longitudinal force
- `M` is the vehicle mass
- `a_des` is the desired acceleration from the PID controller
- `F_resist` is the resistive force

The longitudinal execution chain is:

```text
v_ref, v
  -> PID controller
  -> a_des
  -> longitudinal delay buffer
  -> force-balance model
  -> actuator branch selection: drive / coast / brake
  -> 1D inverse lookup: force to command
  -> pedal publish scaling
  -> 1D forward lookup: command to actual force
  -> coupled_bicycle_dynamics
```

---

## Longitudinal Lookup Logic

The longitudinal actuator model uses a 1D lookup-based structure.

The force demand is first calculated as:

```matlab
F_required = M * a_des + F_resist;
```

Then the model selects one of three actuator branches:

- `drive`
- `coast`
- `brake`

For drive or brake operation, the required force is converted into a map command using 1D interpolation.

The map command is then converted into a published pedal percentage. Finally, the achieved force is re-evaluated using the matching 1D forward map.

This makes the inverse lookup and forward execution consistent with each other.

---

## Pedal Minimum Effective Logic

The model uses map-derived minimum effective pedal thresholds.

The relevant parameters are loaded in:

```matlab
model/load_vehicle_params.m
```

The main thresholds are:

- `veh.acc.pedal_min_publish_from_map`
- `veh.brk.pedal_min_publish_from_map`
- `veh.acc.pedal_min_effective`
- `veh.brk.pedal_min_effective`
- `veh.acc.force_min_effective`
- `veh.brk.force_min_effective`

Current behaviour:

- below the map-derived minimum effective force, inverse lookup returns `0`
- below the map-derived minimum effective command, forward lookup returns `0`
- coast is explicitly represented
- drive/coast/brake switching uses hysteresis

---

## Quick Start

Open MATLAB in the project root and run:

```matlab
main
```

This will:

- load the reference path
- load the reference speed
- load vehicle and actuator map data
- run one closed-loop simulation
- save plots and results under the `run/` folder

---

## Controller Selection

Edit:

```matlab
config/default_config.m
```

Example NMPC + PID setup:

```matlab
cfg.controller.lateral = "mpc_kinematic";
cfg.controller.longitudinal = "pid";
cfg.plant.lateral_model = "dynamic";
```

Alternative plant comparison:

```matlab
cfg.controller.lateral = "mpc_kinematic";
cfg.controller.longitudinal = "pid";
cfg.plant.lateral_model = "bicycle_linear";
```

---

## Available Controllers

### Lateral Controllers

| Controller | Description |
|---|---|
| `stanley` | Heading plus cross-track feedback controller |
| `pure_pursuit` | Geometric lookahead steering controller |
| `mpc` | Dynamic-state lateral MPC |
| `mpc_kinematic` | Kinematic bicycle NMPC with curvature preview |
| `mpc_combined` | Combined steering and acceleration MPC |
| `fake_controller` | Debug controller for subsystem testing |

### Longitudinal Controllers

| Controller | Description |
|---|---|
| `pid` | PID speed controller producing desired acceleration |
| `fake_controller` | Debug controller for subsystem testing |

---

## Outputs

Each simulation run produces output files such as:

| File | Description |
|---|---|
| `result.mat` | Full saved workspace for the run |
| `summary.txt` | Key metrics and pass/fail summary |
| `path_tracking.png` | Reference path compared with simulated path |
| `tracking_errors.png` | Cross-track, heading, longitudinal, and speed errors |
| `speed_tracking.png` | Vehicle speed and longitudinal acceleration |
| `lateral_dynamics.png` | Steering, lateral velocity, and yaw-rate behaviour |
| `execution_timing.png` | Controller execution time |
| `longitudinal_internal_diagnostics.png` | Force demand, actuator branch, and lookup diagnostics |
| `sim_vs_bag.png` | Simulation compared with bag data if available |

---

## Run Modes

### Single Run

```matlab
main
```

Outputs are saved under:

```text
run/single_run/
```

### Lateral MPC Comparison

```matlab
compare_lateral_mpc_variants
```

### Tuning Sweep

```matlab
run_tuning_sweep
```

---

## Practical Tuning Guidance

Recommended tuning order:

1. choose the lateral plant model
2. tune lateral NMPC tracking and smoothness
3. lock the lateral settings
4. tune the longitudinal PID controller

### NMPC Tuning Parameters

Main parameters:

- `cfg.mpc_kinematic.Q`
- `cfg.mpc_kinematic.R`
- `cfg.mpc_kinematic.Rd`
- `cfg.mpc_kinematic.N`
- `cfg.mpc_kinematic.kappa_ff_gain`

General interpretation:

- larger `Q` increases tracking accuracy
- larger `R` reduces steering magnitude
- larger `Rd` smooths steering changes
- larger `N` increases preview horizon
- larger `kappa_ff_gain` increases curvature feedforward contribution

### PID Tuning Parameters

Main parameters:

- `cfg.lon_pid.kp`
- `cfg.lon_pid.ki`
- `cfg.lon_pid.kd`

General interpretation:

- increase `kp` first for stronger speed response
- add `ki` only if there is steady-state speed error
- add `kd` only if the speed response is too oscillatory

The PID should not be tuned as if it directly commands pedal percentage because the actuator and resistance behaviour are handled in the plant model.

---

## Project Structure

```text
Controller_GDP/
  main.m
  compare_lateral_mpc_variants.m
  run_tuning_sweep.m

  config/
    default_config.m

  controllers/
    lateral/
      mpc_lateral.m
      mpc_kinematic_lateral.m
      mpc_combined.m
      pure_pursuit_lateral.m
      stanley_lateral.m
    longitudinal/
      PID_controller.m

  model/
    coupled_bicycle_dynamics.m
    lateral_model.m
    lateral_tire_model.m
    load_vehicle_params.m
    longitudinal_model.m

  plotting/
    save_sim_vs_bag_plot.m
    plot_lookup_maps.m

  reference/
    load_reference_path.m
    load_reference_speed.m
    path_curvature.m
    path_yaw.m
    smooth_speed_profile.m

  simulation/
    run_closed_loop.m
    track_errors.m

  utils/
    angle_wrap.m
    bag_to_ref.m
    nearest_path_ref_point.m
    rate_limit.m
    read_rosbag_kia.m

  data/
    path_ref.mat
    bag_data_10hz.mat
    Acc_mapData_noSlope.mat
    brake_mapData_noSlope.mat
    reference_velocity/

  run/
    single_run/
    compare_run/
```

---

## Dependencies

- MATLAB R2020b or later recommended
- Optimization Toolbox for MPC/QP-based functions
- ROS Toolbox if reading ROS2 bag data directly

---

## Notes

This repository is intended for simulation, controller development, and comparison of NMPC + PID vehicle control behaviour before deployment in the ROS2 vehicle-controller stack.
