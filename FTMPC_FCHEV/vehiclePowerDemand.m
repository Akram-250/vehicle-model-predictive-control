function Pd = vehiclePowerDemand(t, v, params)
%VEHICLEPOWERDEMAND Compute DC bus power demand from speed profile.
%
%   Pd = vehiclePowerDemand(t, v, params) returns the electric power
%   demand at the DC bus for the given speed profile v (m/s). The model
%   includes rolling resistance, aerodynamic drag and inertial effects.
%   Drivetrain efficiencies are included as in the report.

m   = params.mVeh;
g   = params.g;
c_r = params.c_r;
c_d = params.c_d;
A_f = params.A_f;
rho = params.rho_air;
eta = params.eta_mec*params.eta_inv*params.eta_em;

% numerical differentiation for acceleration
acc = [diff(v)./diff(t); 0];

F_inert = m.*acc;
F_roll  = c_r*m*g*ones(size(v));
F_aero  = 0.5*rho*A_f*c_d.*v.^2;
F_total = F_inert + F_roll + F_aero;
P_wheel = F_total.*v;

Pd = P_wheel./eta;   % DC bus power demand

end

