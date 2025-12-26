
Gc1 = K1;
Gc2 = K2;
G11simp = G11;
%% -------------------------------------------------
% 3) Open-loop systems (FULL plant)
%% -------------------------------------------------
L1 = Gc1 * G11;
L2 = Gc2 * G11;

%% -------------------------------------------------
% Open-loop systems (SIMPLIFIED plant)
%% -------------------------------------------------
L1_simp = Gc1 * G11simp;
L2_simp = Gc2 * G11simp;

%% -------------------------------------------------
% 4) Open-loop Bode + margins (FULL plant)
%% -------------------------------------------------
figure
margin(L1)
grid on
title('Open-loop (Full plant), PI controller, f_c = 7 kHz')

figure
margin(L2)
grid on
title('Open-loop (Full plant), PI controller, f_c = 14 kHz')

L3 = Gc2 * G11;

figure
margin(L3)
grid on
title('Open-loop (Full plant), PI controller, f_c = 300 Hz')

%% -------------------------------------------------
% Open-loop Bode + margins (SIMPLIFIED plant)
%% -------------------------------------------------
figure
margin(L1_simp)
grid on
title('Open-loop (Simplified plant), PI controller, f_c = 70 Hz')

figure
margin(L2_simp)
grid on
title('Open-loop (Simplified plant), PI controller, f_c = 700 Hz')

L3_simp = Gc2 * G11simp;

figure
margin(L3_simp)
grid on
title('Open-loop (Simplified plant), PI controller, f_c = 300 Hz')

%% -------------------------------------------------
% 5) Closed-loop current response
%% -------------------------------------------------
T1 = feedback(L1,1);
T2 = feedback(L2,1);

figure
step(T1, T2, 0.2)
grid on
legend('f_c = 7 kHz','f_c = 14kHz','Location','best')
title('Closed-loop Inductor Current Response (Full plant)')

T3 = feedback(L3,1);

figure
step(T1, T3, 0.5)
grid on
legend('7 kHz','300 Hz','Location','best')
title('Closed-loop Inductor Current Response (Full plant)')

%% -------------------------------------------------
% Closed-loop responses (SIMPLIFIED plant)
%% -------------------------------------------------
T1_simp = feedback(L1_simp,1);
T2_simp = feedback(L2_simp,1);

figure
step(T1_simp, T2_simp, 0.2)
grid on
legend('f_c = 70 Hz','f_c = 700 Hz','Location','best')
title('Closed-loop Inductor Current (Simplified plant)')

%% -------------------------------------------------
% 6) Control effort (duty-cycle behavior)
%% -------------------------------------------------
U1 = feedback(Gc1, G11);
U2 = feedback(Gc2, G11);

figure
step(U1, U2, 0.2)
grid on
legend('f_c = 70 Hz','f_c = 700 Hz','Location','best')
title('Duty-cycle (Control Effort) Response – Full plant')

%% -------------------------------------------------
% Control effort (SIMPLIFIED plant)
%% -------------------------------------------------
U1_simp = feedback(Gc1, G11simp);
U2_simp = feedback(Gc2, G11simp);

figure
step(U1_simp, U2_simp, 0.2)
grid on
legend('f_c = 70 Hz','f_c = 700 Hz','Location','best')
title('Duty-cycle Response – Simplified plant')
