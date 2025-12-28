clc; clear; close all;

%% =================================================
% Parameters
%% =================================================
L  = 0.4137e-3;
C  = 40e-6;
RL = 0.03799;
Vb = 750;
fs = 70e3;

w0 = sqrt(1/(L*C));    % LC resonance frequency

s = tf('s');

%% =================================================
% Plant models
%% =================================================
% Duty cycle -> inductor current (full LC model)
den = s^2 + (RL/L)*s + 1/(L*C);
G11 = (Vb/L)*s / den;

% Inductor current -> PV voltage
Gvi = -1/(C*s);

%% =================================================
% INNER CURRENT CONTROLLER
% Target crossover: 7 kHz
% PI zero placed at LC resonance
%% =================================================
fc_i = 7000;
wc_i = 2*pi*fc_i;
wi_i = w0;

% Magnitude of plant at desired crossover
[magG, ~] = bode(G11, wc_i);
magG = squeeze(magG);

% Controller gains
Kp_i = 1/(magG * sqrt(1 + (wi_i/wc_i)^2));
Ki_i = Kp_i * wi_i;

Ci = Kp_i * (1 + wi_i/s);

% Open and closed current loop
L_i = Ci * G11;
T_i = feedback(L_i,1);     % i_L / i_L*

%% ---- Print current controller parameters ----
fprintf('---------------------------------------------\n');
fprintf('INNER CURRENT CONTROLLER\n');
fprintf('Crossover frequency f_ci = %.1f Hz\n', fc_i);
fprintf('PI zero frequency f_zi = %.1f Hz\n', wi_i/(2*pi));
fprintf('Kp_i = %.6f\n', Kp_i);
fprintf('Ki_i = %.6f\n', Ki_i);

[~, PM_i, ~, Wcp_i] = margin(L_i);
fprintf('Phase Margin = %.2f deg at %.1f Hz\n', PM_i, Wcp_i/(2*pi));
fprintf('---------------------------------------------\n\n');

%% =================================================
% OUTER VOLTAGE CONTROLLER
% Target crossover: 700 Hz
%% =================================================
fc_v = 700;
wc_v = 2*pi*fc_v;

wi_v = 0.5 * wc_v;

% Effective plant seen by voltage controller
G_v_eff = Gvi * T_i;

% Plant magnitude at voltage crossover
[magGv, ~] = bode(G_v_eff, wc_v);
magGv = squeeze(magGv);

% Voltage controller gains
Kp_v = 1/(magGv * sqrt(1 + (wc_v/wi_v)^2));
Ki_v = Kp_v * wi_v;

Cv = -Kp_v * (1 + wi_v/s);

%% ---- Print voltage controller parameters ----
fprintf('OUTER VOLTAGE CONTROLLER\n');
fprintf('Crossover frequency f_cv = %.1f Hz\n', fc_v);
fprintf('PI zero frequency f_zv = %.1f Hz\n', wi_v/(2*pi));
fprintf('Kp_v = %.6f\n', Kp_v);
fprintf('Ki_v = %.6f\n', Ki_v);

%% =================================================
% FULL CASCADED SYSTEM
%% =================================================
L_v = Cv * G_v_eff;
T_v = feedback(L_v,1);     % v_pv / v_pv*

[~, PM_v, ~, Wcp_v] = margin(L_v);
fprintf('Phase Margin = %.2f deg at %.1f Hz\n', PM_v, Wcp_v/(2*pi));
fprintf('---------------------------------------------\n');

%% =================================================
% PLOTS
%% =================================================

% Plant
figure;
bode(G11)
grid on;
title('Duty-cycle to Inductor Current Plant');

% Inner current loop
figure;
margin(L_i)
grid on;
title('Inner Current Loop Open-Loop');

% Outer voltage loop (with current loop inside)
figure;
margin(L_v)
grid on;
title('Outer Voltage Loop Open-Loop (Cascaded)');

% Closed-loop poles (entire cascaded system)
figure;
pzmap(T_v)
grid on;
title('Closed-Loop Poles of Full Cascaded System');

% Closed-loop step response
figure;
step(T_v)
grid on;
title('Closed-Loop PV Voltage Step Response');
