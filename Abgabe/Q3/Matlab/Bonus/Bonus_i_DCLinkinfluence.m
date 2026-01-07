% Parameters
L  = 0.4137e-3;
C  = 50e-6;
RL = 0.03799;
Vb = 750;
D  = 0.5;

s = tf('s');


% Small-signal plants 
den = s^2 + (RL/L)*s + 1/(L*C);

% duty -> iL
Gid = (Vb/L)*s/den;

% iL -> vpv 
Gvi = 1/(C*s);

% vb -> vpv  
Gvb = (D/(L*C))/den;


% Controllers
Kp_v = 0.1939;   Ki_v = 426.5086;
Kp_i = 0.0171549; Ki_i = 754.5;

Ci = Kp_i + Ki_i/s;
Cv = Kp_v + Ki_v/s;


% Cascade + disturbance transfer 
Li = Ci*Gid;
Ti = feedback(Li, 1);                 

Lv = Cv*Gvi*Ti;                       
Tvb = Gvb/(1 + Lv);                   

%Outputs
f = 100;  w = 2*pi*f;
mag100 = abs(squeeze(freqresp(Tvb, w)));
mag100_dB = 20*log10(mag100);

fprintf('vb -> vpv closed-loop disturbance transfer\n');
fprintf('Gain at 100Hz = %.6f  (%.2f dB)\n', mag100, mag100_dB);
fprintf('For 10 V - ripple amplitude = %.3f V\n', 10*mag100);

figure;
bode(Tvb); grid on;
title('Closed-loop disturbance transfer: v_{pv}/v_b');
