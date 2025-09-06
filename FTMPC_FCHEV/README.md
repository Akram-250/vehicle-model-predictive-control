# Fault-Tolerant MPC for FCHEV

This folder contains MATLAB scripts reproducing the simulation study from
*Fault-Tolerant Model Predictive Control for Fuel Cell Hybrid Electric Vehicles*.

`runMobypostFTMPC.m` is the main entry point. It generates a stochastic
mail delivery driving cycle, applies the four fault scenarios and compares
the proposed fault-tolerant MPC (FTMPC) with a simplified
Fault-Tolerant Dynamic Programming (FTDP) benchmark. The script prints a
summary table of hydrogen consumption and computation time for each
scenario.

The implementation relies solely on MATLAB/Octave and the Optimization
Toolbox for `quadprog`. For environments without `quadprog`, replace it
with any QP solver compliant with the same interface.

## Files
- `runMobypostFTMPC.m` – main script
- `generateDriveCycle.m` – stochastic drive cycle generator
- `vehiclePowerDemand.m` – vehicle dynamics and power demand calculation
- `simulateScenario.m` – wrapper executing FTMPC and FTDP
- `simulateFTMPC.m` – fault-tolerant MPC controller
- `simulateFTDP.m` – simplified dynamic programming benchmark

## Usage
From MATLAB or Octave, run:
```matlab
runMobypostFTMPC
```

The script outputs a table summarising the hydrogen consumption and
relative performance.

