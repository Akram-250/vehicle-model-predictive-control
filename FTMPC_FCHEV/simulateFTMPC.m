function [H2, logs] = simulateFTMPC(Pd, faults, params)
%SIMULATEFTMPC Fault-tolerant MPC simulation.
%
%   [H2, logs] = simulateFTMPC(Pd, faults, params) simulates the
%   fault‑tolerant MPC controller for the given power demand Pd.
%   The implementation closely follows the equations in Sections 7--9 of
%   the report. The horizon is linearised at each step and solved as a QP
%   using quadprog.

N  = params.N;
dt = params.dt;
Hp = params.Hp;
Hc = params.Hc;

SoC = params.SoC0;
Pfc = 0;                     % previous fuel cell power
H2  = 0;                     % accumulated hydrogen [g]

% cost weights
w1 = 1; w2 = 1; w3 = 10;

logs.Pfc = zeros(N,1);
logs.SoC = zeros(N,1);

for k = 1:N
    % --- fault‑dependent parameters ---
    Qbat = params.Qbat*(1 - faults.delta_bat);
    Rbat = params.Rbat(SoC)*(1 + faults.epsilon_bat);
    Voc  = params.Voc(SoC);
    Vdc  = Voc - 0;   % neglect I*R for linearisation
    etabat = params.etabat;

    Pfc_max = params.Pfc_max*(1 - faults.beta_fc);
    dPfc_max = params.Pfc_rate*(1 - faults.alpha_fc);
    eta_fc = @(P) params.eta_fc(P).*(1 - faults.alpha_fc);

    % build linear model matrices (A,B,Bw)
    A = [1 dt*etabat*params.eta_inv/(Vdc*Qbat); 0 1];
    B = [dt^2*etabat*params.eta_inv/(Vdc*Qbat); dt];
    Bw= [-dt*etabat/(Vdc*Qbat); 0];

    % prediction vectors
    Pd_pred = Pd(k:min(k+Hp-1,N));
    Pd_pred = [Pd_pred; Pd_pred(end)*ones(Hp-numel(Pd_pred),1)];

    % matrices for prediction
    [Phi, Psi_u, Psi_w] = buildPrediction(A,B,Bw,Hp,Hc);
    xk = [SoC; Pfc];

    % reference trajectories
    Pfc_ref = zeros(Hp,1);   % prefer low fuel cell usage
    Xref = [params.SoCref*ones(Hp,1) Pfc_ref];

    % cost matrices
    Q = blkdiag(kron(eye(Hp-1),diag([w3 w1])),diag([w3 w1]));
    R = w2*eye(Hc);

    H = 2*(Psi_u'*Q*Psi_u + R);
    f = 2*Psi_u'*Q*(Phi*xk + Psi_w*Pd_pred - Xref(:));

    % inequality constraints
    % fuel cell power bounds
    A1 = Psi_u(2:2:end,:);  % rows corresponding to Pfc predictions
    b1 = (Pfc_max - Phi(2:2:end,:)*xk - Psi_w(2:2:end,:)*Pd_pred);

    % rate constraints
    A2 = [eye(Hc); -eye(Hc)];
    b2 = [dPfc_max*ones(Hc,1); dPfc_max*ones(Hc,1)];

    % aggregate
    Aineq = [A1; -A1; A2];
    bineq = [b1; Pfc_max*ones(size(b1)) - b1; b2];

    % solve QP
    options = optimoptions('quadprog','Display','off');
    du = quadprog(H,f,Aineq,bineq,[],[],[],[],[],options);
    du = du(1);   % apply only first control move

    % update states
    Pfc = Pfc + dt*du;
    Pfc = max(0,min(Pfc,Pfc_max));
    Pbat = Pd(k) - Pfc*params.eta_inv;
    Ibat = Pbat / max(Vdc,1);
    SoC = SoC - dt*etabat*Ibat/Qbat;

    % hydrogen consumption
    H2 = H2 + dt*Pfc ./ max(eta_fc(Pfc)*params.rhoH2,1);

    logs.Pfc(k) = Pfc;
    logs.SoC(k) = SoC;
end

H2 = H2*1000;   % convert kg to g
end

function [Phi, Psi_u, Psi_w] = buildPrediction(A,B,Bw,Hp,Hc)
% Construct prediction matrices for time-invariant system
n = size(A,1);
Phi = zeros(n*Hp,n);
Psi_u = zeros(n*Hp,Hc);
Psi_w = zeros(n*Hp,Hp);

for i=1:Hp
    Phi((i-1)*n+1:i*n,:) = A^i;
    for j=1:min(i,Hc)
        Psi_u((i-1)*n+1:i*n,j) = A^(i-j)*B;
    end
    for j=1:i
        Psi_w((i-1)*n+1:i*n,j) = A^(i-j)*Bw;
    end
end
end

