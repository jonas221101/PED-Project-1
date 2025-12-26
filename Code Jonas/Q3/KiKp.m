clear all; close all; clc;

s = tf('s');
Lval = 0.4137e-3;
C = 40e-6;
RL = 0.03799;
Vb = 750;

G11 = -(Vb/Lval)*s / (s^2 + (RL/Lval)*s + 1/(Lval*C));

fs = 70e3;
wc_list = [2*pi*(fs/5), 2*pi*(fs/10)];
PM_des = 50;

% Parameter-Suchraum
Kp_range = -logspace(-1, 4, 100);
Ki_range = -logspace(-1, 5, 100);

Kp_res = zeros(1,2);
Ki_res = zeros(1,2);

for idx = 1:2
    
    wc = wc_list(idx);
    best_error = inf;
    
    for i = 1:length(Kp_range)
        for j = 1:length(Ki_range)

            Kp = Kp_range(i);
            Ki = Ki_range(j);
            K = Kp + Ki/s;

            L = K*G11;

            [mag, phase_deg] = bode(L, wc);
            mag = mag(:); 
            phase_deg = phase_deg(:);

            % Magnitude error for |L(j wc)| = 1
            err_mag = abs(mag - 1);

            % Phase margin
            PM = 180 + phase_deg; 
            err_pm = abs(PM - PM_des);

            % Combined error
            err = err_mag + 0.1*err_pm;

            if err < best_error
                best_error = err;
                Kp_res(idx) = Kp;
                Ki_res(idx) = Ki;
            end

        end
    end
end

Kp1 = Kp_res(1)
Ki1 = Ki_res(1)

Kp2 = Kp_res(2)
Ki2 = Ki_res(2)

K1 = Kp1 + Ki1/s;
K2 = Kp2 + Ki2/s;

T1 = feedback(K1*G11, 1);
T2 = feedback(K2*G11, 1);

figure;
step(T1); hold on;
step(T2);
legend("fc = fs/5", "fc = fs/10");
grid on;
%bode(T1,T2);

Gc1 = K1;
Gc2 = K2;
G11simp = G11;
