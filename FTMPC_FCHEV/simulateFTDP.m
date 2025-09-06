function H2 = simulateFTDP(Pd, faults, params)
%SIMULATEFTDP Simplified fault‑tolerant Dynamic Programming benchmark.
%
%   H2 = simulateFTDP(Pd, faults, params) computes the globally optimal
%   hydrogen consumption for the given demand profile using a discrete DP
%   approach. The implementation is simplified for clarity and follows the
%   structure of Section 10 in the report.

N = params.N;
dt = params.dt;

% fault‑dependent limits
SoCmin = params.SoCmin + 0.05*sqrt(faults.delta_bat^2 + faults.epsilon_bat^2);
SoCmax = params.SoCmax - 0.05*sqrt(faults.delta_bat^2 + faults.epsilon_bat^2);
Pfc_max = params.Pfc_max*(1 - faults.beta_fc);

a = 0:50:Pfc_max;                 % control grid (fuel cell power)
S = linspace(SoCmin, SoCmax, 61); % SoC grid
ns = numel(S); na = numel(a);

V = inf(ns, N+1);
V(:,N+1) = (S - params.SoCref).^2;  % terminal cost

for k = N:-1:1
    for i = 1:ns
        soc = S(i);
        for j = 1:na
            pfc = a(j);
            [socn, h2, feasible] = batteryUpdate(soc, Pd(k), pfc, faults, params);
            if ~feasible || socn < SoCmin || socn > SoCmax
                continue; % infeasible move
            end
            idx = interp1(S,1:ns,socn,'nearest');
            cost = h2 + V(idx,k+1);
            if cost < V(i,k)
                V(i,k) = cost;
            end
        end
    end
end

idx0 = interp1(S,1:ns,params.SoC0,'nearest');
H2 = V(idx0,1)*1000;  % g of hydrogen
end

function [socn, h2, feasible] = batteryUpdate(soc, Pd, Pfc, faults, params)
% batteryUpdate compute next SoC and hydrogen consumption for given control
Qbat = params.Qbat*(1 - faults.delta_bat);
Voc  = params.Voc(soc);
Vdc  = Voc;
eta  = params.etabat;

Pbat = Pd - Pfc*params.eta_inv;
Ibat = Pbat / max(Vdc,1);
socn = soc - params.dt*eta*Ibat/Qbat;

eta_fc = params.eta_fc(Pfc).*(1 - faults.alpha_fc);
if eta_fc <= 0
    feasible = false;
    h2 = inf;
else
    feasible = true;
    h2 = params.dt*Pfc/(eta_fc*params.rhoH2);
end
end

