%Crossover frequency at 7 kHz

%Parameters
L  = 0.4137e-3;
C  = 40e-6;
RL = 0.03799;
fs = 70e3;
Vb = 750;
D  = 0.5;

%% -------------------------------------------------
% Plant models
%% -------------------------------------------------
s = tf('s');

% Full LC plant (duty -> inductor current)
den = s^2 + (RL/L)*s + 1/(L*C);
G11 = (Vb/L)*s / den;

% Simplified RL plant
G11simp = Vb/(s*L + RL);

figure
bode(G11, G11simp)
grid on
legend('Full LC plant','Simplified RL plant','Location','best')
title('Duty-cycle to Inductor Current Transfer Function')

fc1 = 400;       
wc1 = 2*pi*fc1;

wi1 = wc1; 
Kp1 = wc1*L/(sqrt(2)*Vb); 
Ki1 = Kp1 * wi1;
fprintf('400 Hz controller:\n');
fprintf('  Kp1 = %.6g\n', Kp1);
fprintf('  Ki1 = %.6g\n', Ki1);
Gc1 = Kp1*(1 + wi1/s);

L1 = Gc1 * G11;
L1_simp = Gc1 * G11simp;

figure
margin(L1)
hold on
margin(L1_simp)
grid on
legend('Full LC plant', 'Simplified RL plant', 'Location','best')
title('Open-loop Bode with Margins, PI controller, f_c = 400 kHz')