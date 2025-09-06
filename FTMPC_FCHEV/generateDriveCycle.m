function [t, v] = generateDriveCycle(params)
%GENERATEDRIVECYCLE Creates stochastic mail delivery speed profile.
%
%   [t,v] = generateDriveCycle(params) returns time vector t and speed
%   profile v (m/s) for the duration specified in params.N. The model
%   follows the description in Section 4.1 of the report.
%
%   The speed profile consists of a base cruise speed with sinusoidal
%   variation, additive Gaussian noise, and random stops of duration
%   30-60 s every 120-180 s.

T = params.N;          % total simulation time [s]
dt = params.dt;

% Parameters from the report
vCruise = 35/3.6;      % 35 km/h in m/s
Tcycle  = 300;         % cycle period [s]
noiseStd = 2/3.6;      % speed variation [m/s]

% Preallocate
n = T/dt;
t = (0:n-1)'*dt;
v = vCruise*(1 + 0.2*sin(2*pi*t/Tcycle)) + noiseStd*randn(n,1);

% Generate stop times
nextStop = 120 + 60*rand;
idx = 1;
while idx <= n
    if t(idx) >= nextStop
        dur = 30 + 30*rand;
        stopIdx = idx:min(n, idx+round(dur/dt));
        v(stopIdx) = 0;
        nextStop = nextStop + 120 + 60*rand;  % schedule next stop
        idx = stopIdx(end);
    end
    idx = idx + 1;
end

% ensure non-negative speeds
v(v<0) = 0;
end

