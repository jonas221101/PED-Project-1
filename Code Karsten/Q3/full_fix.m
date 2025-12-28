clc; clear; close all;

%% =========================
% Parameters
%% =========================
L  = 0.4137e-3;
C  = 40e-6;
RL = 0.03799;
Vb = 750;
fs = 70e3;

s = tf('s');

%% =========================
% Plant
%% =========================
den = s^2 + (RL/L)*s + 1/(L*C);
Gid = (Vb/L)*s / den;     % duty -> inductor current
Gvi = -1/(C*s);           % current -> PV voltage

%% =========================
% INNER CURRENT CONTROLLER (7 kHz)
%% =========================
fci = 7e3;
wci = 2*pi*fci;

wi_i = wci;
Kp_i = wci*L/(sqrt(2)*Vb);
Ki_i = Kp_i*wi_i;

Ci = Kp_i*(1 + wi_i/s);

% Close current loop
Li = Ci * Gid;
Ti = feedback(Li,1);      % iL / iL*

%% =========================
% OUTER VOLTAGE CONTROLLER (700 Hz)
%% =========================
fcv = 700;
wcv = 2*pi*fcv;

% PI zero
wi_v = 0.5*wcv;

% HF stabilising pole (KEY FIX)
wp_v = 10*wcv;

% Gains
Kp_v = 2*C*wcv / sqrt(1 + (wcv/wi_v)^2);
Ki_v = Kp_v * wi_v;

% Voltage controller (PI + pole)
Cv = -Kp_v * (1 + wi_v/s) * (1 / (1 + s/wp_v));

%% =========================
% FULL CASCADED SYSTEM
%% =========================
Gv_eff = Gvi * Ti;        % v_pv / iL*
Lv = Cv * Gv_eff;         % voltage open-loop
Tv = feedback(Lv,1);      % v_pv / v_pv*

%% =========================
% ANALYSIS
%% =========================

% Inner loop
figure;
margin(Li)
title('Inner Current Loop Open-Loop');
grid on;

% Voltage loop (with stabilisation)
figure;
margin(Lv)
title('Outer Voltage Loop Open-Loop (STABLE)');
grid on;

% Nyquist (THIS is the proof)
figure;
nyquist(Lv)
grid on;
title('Nyquist Plot of Voltage Loop (No -1 Encirclement)');

% Closed-loop poles
figure;
pzmap(Tv)
grid on;
title('Closed-Loop Poles of Full Cascaded System');

% Step response
figure;
step(Tv)
grid on;
title('Closed-Loop PV Voltage Step Response');
