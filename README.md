# AGEVAR - Multi-Module Robot Kinematics Simulation

This repository contains the kinematics simulation for a multi-module articulated robot.

## Repository Structure

### Main Branch (`main`)

- **File**: `agevar.m`
- **Purpose**: Development and testing version
- **Features**: Single simulation run, configurable parameters at the top
- **Usage**: Ideal for development, testing new features, and academic work

### Demo Branch (`demo-loop`)

- **File**: `agevar.m`
- **Purpose**: Continuous demo version for fairs and presentations
- **Features**: Infinite loop through all trajectories and module configurations
- **Usage**: Perfect for demonstrations, fairs, and continuous displays

## How to Use

### For Development Work

```bash
git checkout main
# Edit parameters in agevar.m:
# modules = 2;           % Number of modules [2, 4, 6]
# trajectory_id = 1;     % 1-5 for different trajectories
```

### For Demonstrations

```bash
git checkout demo-loop
# Run agevar.m - it will loop continuously through all configurations
```

### Pulling Updates

To pull updates from main branch to demo-loop branch:

```bash
git checkout demo-loop
git merge main
```

## Robot Parameters

- **WheelSpan**: 0.210 m - Distance between wheels
- **Radius**: 0.0605 m - Wheel radius
- **Module Length**: ~0.331 m - Total module length
- **Connection distances**: a=0.2015 m, b=0.209 m

## Trajectories Available

1. **Simple case**: Constant velocity circular arc
2. **Simple case smooth_w**: Smooth angular velocity ramp
3. **S-curve**: S-shaped path with alternating turns
4. **U-curve**: U-turn maneuver
5. **<3 shape**: Heart-shaped trajectory

## Module Configurations

- 2 modules: Basic articulated robot
- 4 modules: Extended articulated robot
- 6 modules: Extreme-length articulated robot

## Example

![ReseQ gif](ReseQ.gif)
