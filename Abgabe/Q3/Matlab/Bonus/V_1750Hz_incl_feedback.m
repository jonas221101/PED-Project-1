clear all;

syms   Kp real
C = 50e-6;
w = 1750 *2 * pi;
% Closed-loop current controller
T = (12.87*1i*w + 5.659e5)/(-0.0004137*w^2 + 12.9*1i*w + 5.659e5);

% Voltage plant
Gvi = -1/(C*1i*w) * T;

% PI zero (place at 0.5*w)
wi = 0.5*w;
Ki = Kp * wi;

% Open loop
Gopen = (Kp + Ki/(1i*w)) * Gvi;

% Magnitude condition |Gopen| = 1  --> use |G|^2 = Re^2 + Im^2
eqn = real(Gopen)^2 + imag(Gopen)^2 == 1;

% Solve Kp
Kp_solution = double(solve(eqn, Kp))
Ki_Solution = double(Kp_solution * wi)

PI_current_7kHz;
clc; close all;

%% Parameters
C = 50e-6;
s = tf('s');
Ti = feedback(Gc1 * G11simp, 1);

% Plant (sign absorbed in feedback)
Gvi = -1/(C*s) * Ti;
Cv1 = -Kp_solution(2) * (1 + wi/s);
L1  = Cv1 * Gvi;

[~, PM1, ~, Wcp1] = margin(L1);

fprintf('--- Voltage loop @ 700 Hz ---\n');
fprintf('Kp = %.4f\n', Kp_solution(2));
fprintf('Ki = %.4f\n', Ki_Solution(2));
fprintf('PM = %.2f deg\n', PM1);
fprintf('Crossover = %.1f Hz\n\n', Wcp1/(2*pi));

figure;
margin(L1);
title('Voltage Loop Open-Loop Bode (700 Hz)');
grid on;

figure ;
pzmap(L1)
grid on;
title("Closed Loop Poles");