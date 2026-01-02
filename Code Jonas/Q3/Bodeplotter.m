L = 0.4137e-3;
C = 50e-6%2.16e-3;
RL = 0.03799;
fs = 70e3;
Vb= 750;
D = 0.5;

s = tf('s');
den = s^2 + (RL/L)*s + 1/(L*C);
G11 = -(Vb/L)*s / den;
G12 = -(D/L)*s  / den;
G21 =  (Vb/(L*C)) / den;
G22 =  (D/(L*C))  / den;

G = [G11 G12;
     G21 G22];

G11simp = Vb/(s*L+RL)

figure
bode(G11, G11simp)
grid on
title('Bode Plot of G(s)')
