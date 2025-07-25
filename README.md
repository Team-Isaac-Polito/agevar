# AGEVAR - Multi-Module Robot Kinematics Simulation

## Main Branch

**File**: `agevar.m`

**Purpose**: Development and testing version for multi-module articulated robot simulation.

**Features**:

- Configurable number of modules (2, 4, 6)
- 5 different trajectory types: Simple case, Simple case smooth_w, S-curve, U-curve, Heart shape (<3)
- Kinematic coupling between modules via passive joints
- Real-time visualization and animation

**Configuration**: Edit parameters at the top of `agevar.m`:

```matlab
modules = 4;                    % Number of modules [2, 4, 6]
trajectory_id = 3;              % 1-5 for different trajectories
```

## Other Branches

- **demo-loop**: Continuous demo version that loops through all configurations
- **pivot**: Sequential pivot maneuver simulation
- **slip-control**: Advanced slip control algorithms

![Robot Simulation](ReseQ.gif)
