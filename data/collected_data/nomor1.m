% =========================================================================
% MATLAB Script: Command Generation for Flexible Systems
% Plotting Step Responses and Sensitivity Curves untuk Soal 1a, 1b, & 1c
% =========================================================================

clear; clc; close all;

%% 1. DEFINISI SISTEM (PLANT)
% G(s) = 36 / ((s^2 + 4)*(s^2 + 9)) = 36 / (s^4 + 13s^2 + 36)
num = 36;
den = [1 0 13 0 36];   % <--- DIPERBAIKI: s^4 + 0*s^3 + 13*s^2 + 0*s + 36
G = tf(num, den);

% Parameter Simulasi Waktu
t = 0:0.01:8;   % Simulasi dari 0 hingga 8 detik

%% 2. PARAMETER INPUT SHAPER
% --- 1(a) Combined ZV Shaper ---
A_1a = [0.25, 0.25, 0.25, 0.25];
t_1a = [0, pi/3, pi/2, 5*pi/6];

% --- 1(b) Simultaneous ZV Shaper ---
A_1b = [(5-sqrt(5))/10, sqrt(5)/5, (5-sqrt(5))/10];
t_1b = [0, 2*pi/5, 4*pi/5];

% --- 1(c) UM-ZV Shaper (Baru) ---
% Optimasi numerik untuk mencari t2, t3, t4, t5
A_1c = [1, -1, 1, -1, 1];
T0 = [0.5, 1.0, 1.5, 2.0]; % Tebakan awal
A_ineq = [1 -1 0 0; 0 1 -1 0; 0 0 1 -1]; % t2 < t3 < t4 < t5
b_ineq = [0; 0; 0];
lb = [0; 0; 0; 0]; % Batas bawah waktu (harus positif)
options = optimoptions('fmincon', 'Display', 'off');
% Mencari waktu dengan meminimalkan waktu impuls terakhir (T(4))
T_opt = fmincon(@(T) T(4), T0, A_ineq, b_ineq, [], [], lb, [], @umzv_con, options);
t_1c = [0, T_opt];

%% 3. MEMBUAT PROFIL PERINTAH (SHAPED STEP INPUTS)
% Unshaped Input (Unit Step biasa)
u_unshaped = ones(size(t));

% Shaped Input 1(a) - Bentuk Anak Tangga (Staircase)
u_1a = zeros(size(t));
for i = 1:length(A_1a)
    u_1a = u_1a + A_1a(i) * (t >= t_1a(i));
end

% Shaped Input 1(b) - Bentuk Anak Tangga (Staircase)
u_1b = zeros(size(t));
for i = 1:length(A_1b)
    u_1b = u_1b + A_1b(i) * (t >= t_1b(i));
end

% Shaped Input 1(c) - Unity Magnitude
u_1c = zeros(size(t));
for i = 1:length(A_1c)
    u_1c = u_1c + A_1c(i) * (t >= t_1c(i));
end

%% 4. SIMULASI RESPONS SISTEM (TIME DOMAIN)
y_unshaped = lsim(G, u_unshaped, t);
y_1a = lsim(G, u_1a, t);
y_1b = lsim(G, u_1b, t);
y_1c = lsim(G, u_1c, t);

% --- Plot 1: Step Responses ---
figure('Name', 'System Responses', 'Color', 'w');
plot(t, y_unshaped, 'k--', 'LineWidth', 1.2); hold on;
plot(t, y_1a, 'b-', 'LineWidth', 2);
plot(t, y_1b, 'r-', 'LineWidth', 2);
plot(t, y_1c, 'm-', 'LineWidth', 2);
yline(1.0, 'g:', 'Setpoint', 'LineWidth', 1);

title('Respons Sistem terhadap Berbagai Input');
xlabel('Waktu (detik)');
ylabel('Amplitudo Posisi');
legend('Tanpa Shaper (Vibrasi Tinggi)', '1(a) Combined ZV (0% Vibrasi)', ...
    '1(b) Simultaneous ZV (0% Vibrasi)', '1(c) UM-ZV (0% Vibrasi)', 'Location', 'Southeast');
grid on;
xlim();   % <--- DIPERBAIKI: beri rentang sumbu-x

%% 5. PERHITUNGAN KURVA SENSITIVITAS (FREQUENCY DOMAIN)
% Menggunakan Persamaan 3.1 dari buku (Vibrasi Sisa untuk zeta = 0)
w = 0:0.05:6;   % Rentang frekuensi pengujian (rad/s)
V_1a = zeros(size(w));
V_1b = zeros(size(w));
V_1c = zeros(size(w));

for j = 1:length(w)
    % Evaluasi untuk 1(a)
    C_1a = sum(A_1a .* cos(w(j) * t_1a));
    S_1a = sum(A_1a .* sin(w(j) * t_1a));
    V_1a(j) = sqrt(C_1a^2 + S_1a^2) * 100;   % dikali 100 untuk persentase

    % Evaluasi untuk 1(b)
    C_1b = sum(A_1b .* cos(w(j) * t_1b));
    S_1b = sum(A_1b .* sin(w(j) * t_1b));
    V_1b(j) = sqrt(C_1b^2 + S_1b^2) * 100;
    
    % Evaluasi untuk 1(c)
    C_1c = sum(A_1c .* cos(w(j) * t_1c));
    S_1c = sum(A_1c .* sin(w(j) * t_1c));
    V_1c(j) = sqrt(C_1c^2 + S_1c^2) * 100;
end

% --- Plot 2: Sensitivity Curves ---
figure('Name', 'Sensitivity Curves', 'Color', 'w');
plot(w, V_1a, 'b-', 'LineWidth', 2); hold on;
plot(w, V_1b, 'r-', 'LineWidth', 2);
plot(w, V_1c, 'm-', 'LineWidth', 2);

% Tandai frekuensi natural sistem (w1 = 2, w2 = 3) dengan garis vertikal
% <--- DIPERBAIKI: dulu plot(,,...) kosong
xline(2, 'k:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
xline(3, 'k:', 'LineWidth', 1.5, 'HandleVisibility', 'off');

text(2.1, 80, '\omega_1 = 2 rad/s');
text(3.1, 80, '\omega_2 = 3 rad/s');

title('Kurva Sensitivitas (Sensitivity Curves)');
xlabel('Frekuensi \omega (rad/s)');
ylabel('Persentase Vibrasi Sisa V(\omega) (%)');
legend('1(a) Combined ZV', '1(b) Simultaneous ZV', '1(c) UM-ZV', 'Location', 'Northwest');
grid on;
ylim();   % <--- DIPERBAIKI: beri rentang sumbu-y

%% FUNGSI LOKAL UNTUK OPTIMASI (Taruh di akhir file)
function [c, ceq] = umzv_con(T)
    % Non-linear constraints untuk 1(c) UM-ZV
    % Memastikan vibrasi sisa = 0 pada w1=2 dan w2=3
    c = []; % Tidak ada non-linear inequality
    ceq = [
        1 - cos(2*T(1)) + cos(2*T(2)) - cos(2*T(3)) + cos(2*T(4));
       -sin(2*T(1)) + sin(2*T(2)) - sin(2*T(3)) + sin(2*T(4));
        1 - cos(3*T(1)) + cos(3*T(2)) - cos(3*T(3)) + cos(3*T(4));
       -sin(3*T(1)) + sin(3*T(2)) - sin(3*T(3)) + sin(3*T(4))
    ];
end