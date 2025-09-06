% runMobypostFTMPC.m
% Main script to reproduce FTMPC vs FTDP results for Mobypost FCHEV
%
% This script generates the stochastic driving cycle, applies the fault
% scenarios from the technical report and evaluates the fault-tolerant
% MPC (FTMPC) controller against the Fault‑Tolerant Dynamic Programming
% (FTDP) benchmark. The script collects hydrogen consumption and
% computation time for each scenario and prints a table similar to
% Table~3 in the report.
%
% This implementation follows the derivations provided in the report and
% assumes the Optimization Toolbox (quadprog) is available.

clear; clc;

%% Simulation parameters
params.dt      = 1;       % sample time [s]
params.N       = 1800;    % mission duration [s]
params.Hp      = 15;      % prediction horizon
params.Hc      = 5;       % control horizon
params.SoC0    = 0.45;    % initial state of charge
params.SoCref  = 0.30;    % terminal reference

% Vehicle and component data (subset of full table for brevity)
params.mVeh    = 579;     % curb mass [kg]
params.c_r     = 0.015;   % rolling resistance
params.c_d     = 0.70;    % drag coefficient
params.A_f     = 2.48;    % frontal area [m^2]
params.rho_air = 1.26;    % air density [kg/m^3]
params.g       = 9.81;    % gravity [m/s^2]
params.eta_mec = 0.92;    % mechanical efficiency
params.eta_inv = 0.95;    % inverter efficiency
params.eta_em  = 0.92;    % motor efficiency

% Battery model parameters
params.Qbat    = 5.5*3600;           % capacity [As]
params.Voc     = @(soc) 280 + 40*soc;% open circuit voltage [V]
params.Rbat    = @(soc) 0.1 + 0.05*(1-soc); % internal resistance [Ohm]
params.etabat  = 0.95;               % coulombic efficiency
params.SoCmin  = 0.30;               % normal range
params.SoCmax  = 0.90;

% Fuel cell parameters
params.Pfc_max = 1200;    % [W]
params.Pfc_rate = 40;     % [W/s]
params.eta_fc = @(P) (P>0).*0.43.*(1 - 0.8*(P/1200 - 550/1200).^2);
params.rhoH2   = 120e6;   % J/kg energy density

%% Generate driving cycle
[time, speed] = generateDriveCycle(params);
Pd = vehiclePowerDemand(time, speed, params);  % wheel power demand

%% Define fault scenarios (Normal, Mild, Moderate, Severe)
scenarios = {
    struct('name','Normal','alpha_fc',0,'beta_fc',0,'delta_bat',0,'epsilon_bat',0),
    struct('name','Mild',   'alpha_fc',0.10,'beta_fc',0,'delta_bat',0,'epsilon_bat',0.05),
    struct('name','Moderate','alpha_fc',0.15,'beta_fc',0.20,'delta_bat',0.15,'epsilon_bat',0.25),
    struct('name','Severe', 'alpha_fc',0.25,'beta_fc',0.30,'delta_bat',0.25,'epsilon_bat',0.40)
    };

results = struct();

for i = 1:numel(scenarios)
    faults = scenarios{i};
    fprintf('Simulating scenario: %s\n', faults.name);
    [mpcRes, dpRes] = simulateScenario(Pd, faults, params);
    results.(faults.name).MPC = mpcRes;
    results.(faults.name).DP  = dpRes;
end

%% Display results table
fprintf('\nPerformance Comparison (H2 consumption in g)\n');
fprintf('%10s %12s %12s %12s %12s\n','Scenario','MPC','DP','Perf(%)','Time ratio');
scnNames = fieldnames(results);
for i = 1:numel(scnNames)
    sc = results.(scnNames{i});
    perf = 100*sc.MPC.H2 / sc.DP.H2;
    timeRatio = sc.MPC.time / sc.DP.time;
    fprintf('%10s %12.1f %12.1f %12.1f %12.3f\n',scnNames{i},sc.MPC.H2,sc.DP.H2,perf,timeRatio);
end

