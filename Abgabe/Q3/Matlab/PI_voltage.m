clc; clear; close all;

%% Parameters
C = 50e-6;
s = tf('s');
%Ti = feedback(Gc1 * G11simp, 1);

% Plant (sign absorbed in feedback)
Gvi = -1/(C*s);
PM_des = 60;   % desired phase margin

%% -------- Design 1: fc = 700 Hz --------
fc1 = 700;
wc1 = 2*pi*fc1;

% PI zero for 60 deg PM
wi1 = wc1*0.5;

% Gains
%Kp1 =  2*C*wc1 / sqrt(1 + (wc1/wi1)^2);
Kp1 =  1*C*wc1 / sqrt(1 + (wi1/wc1)^2);

Ki1 =  Kp1 * wi1;

Cv1 = -Kp1 * (1 + wi1/s);
L1  = Cv1 * Gvi;

[~, PM1, ~, Wcp1] = margin(L1);

fprintf('--- Voltage loop @ 700 Hz ---\n');
fprintf('Kp = %.4f\n', Kp1);
fprintf('Ki = %.4f\n', Ki1);
fprintf('PM = %.2f deg\n', PM1);
fprintf('Crossover = %.1f Hz\n\n', Wcp1/(2*pi));

figure;
margin(L1);
title('Voltage Loop Open-Loop Bode (700 Hz)');
grid on;

%% -------- Design 2: fc = 1750 Hz --------
fc2 = 7000/4;
wc2 = 2*pi*fc2;

wi2 = wc2*0.5;

Kp2 = 2*C*wc2 / sqrt(1 + (wc2/wi2)^2);
Ki2 = Kp2 * wi2;

Cv2 = -Kp2 * (1 + wi2/s);
L2  = Cv2 * Gvi;

[~, PM2, ~, Wcp2] = margin(L2);

fprintf('--- Voltage loop @ 1750 Hz ---\n');
fprintf('Kp = %.4f\n', Kp2);
fprintf('Ki = %.4f\n', Ki2);
fprintf('PM = %.2f deg\n', PM2);
fprintf('Crossover = %.1f Hz\n\n', Wcp2/(2*pi));

figure;
margin(L2);
title('Voltage Loop Open-Loop Bode (1400 Hz)');
grid on;
