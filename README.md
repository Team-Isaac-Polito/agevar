# AGEVAR - Multi-Module Robot Kinematics Simulation

## Pivot Branch

**File**: `agevar.m`

**Purpose**: Sequential pivot maneuver simulation.

**Features**: 
- Sequential pivot behavior: each module pivots in turn
- Constraint-based positioning during pivot maneuvers
- Forward movement maintained during pivot
- Configurable number of modules with automatic phase generation

**Simulation Phases**: 
1. All modules move straight
2. Module 1 pivots while others follow
3. Module 2 pivots using constraint-based control
4. Module 3 pivots using constraint-based control
5. (continues for additional modules)
6. All modules return to straight motion

## Other Branches

- **main**: Development and testing version with trajectory simulation
- **demo-loop**: Continuous demo version for presentations
- **slip-control**: Advanced slip control algorithms

![Robot Simulation](ReseQ.gif)
