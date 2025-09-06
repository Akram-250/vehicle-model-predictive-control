function [mpcRes, dpRes] = simulateScenario(Pd, faults, params)
%SIMULATESCENARIO Execute FTMPC and FTDP for a given fault scenario.
%
%   [mpcRes, dpRes] = simulateScenario(Pd, faults, params) runs the
%   FTMPC controller and FTDP benchmark for the power demand profile Pd
%   under the fault levels specified in faults.

% ---- FTMPC ------------------------------------------------------------
mpcStart = tic;
[H2_mpc, ~] = simulateFTMPC(Pd, faults, params);
mpcRes.H2   = H2_mpc;
mpcRes.time = toc(mpcStart);

% ---- FTDP -------------------------------------------------------------
dpStart = tic;
[H2_dp] = simulateFTDP(Pd, faults, params);
dpRes.H2   = H2_dp;
dpRes.time = toc(dpStart);

end

