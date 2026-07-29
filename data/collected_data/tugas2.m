%% =====================================================================
%  TUGAS 2 : ADAPTIVE SLIDING MODE CONTROL + FUZZY COMPENSATOR
%            Robot manipulator 2-sendi.  Integrasi numerik: ode45.
%  Kompatibel MATLAB R2024b.
%  State vector (14x1): X = [q1; dq1; q2; dq2; W1(1..5); W2(1..5)]
%% =====================================================================
clear; clc; close all;

%% ----------------- Parameter fisik plant -----------------
P.m1 = 1.0;  P.m2 = 1.5;        % massa link (kg)
P.l1 = 1.0;  P.l2 = 0.8;        % panjang link (m)
P.g  = 9.81;                    % gravitasi (m/s^2)

%% ----------------- Parameter fuzzy compensator -----------------
P.centers = [-pi/6; -pi/12; 0; pi/12; pi/6];  % titik tengah Gaussian (5)
P.sigma   = pi/24;                            % lebar Gaussian

%% ----------------- Parameter kontroler -----------------
P.K     = diag([10 10]);     % gain sliding surface
P.KD    = diag([20 20]);     % gain feedback
P.Wrob  = diag([1.5 1.5]);   % term robust (switching)
P.Gamma = 10000;             % learning rate adaptif (1/0.0001)

%% ----------------- Kondisi awal -----------------
% q1=q2=pi/3, dq=0, bobot fuzzy awal 0.10 tiap elemen
X0 = [pi/3; 0; pi/3; 0; 0.10*ones(10,1)];

%% ----------------- Simulasi -----------------
tspan = [0 10];   % 10 detik
opt   = odeset('RelTol',1e-4,'AbsTol',1e-5,'MaxStep',0.005);
[t, X] = ode45(@(t,x) robot_ode(t,x,P), tspan, X0, opt);

%% ----------------- Hitung ulang sinyal untuk plot -----------------
N   = numel(t);
TAU = zeros(N,2);  FF = zeros(N,2);  FH = zeros(N,2);
for k = 1:N
    [~, tau, F, Fhat] = robot_ode(t(k), X(k,:).', P);
    TAU(k,:) = tau.';
    FF(k,:)  = F.';
    FH(k,:)  = Fhat.';
end
qd  = 0.3*sin(t);     % q1d = q2d
dqd = 0.3*cos(t);     % dq1d = dq2d

%% ----------------- Plot -----------------
% Figure 1 : tracking posisi
figure('Name','Tracking Posisi');
subplot(2,1,1)
plot(t, X(:,1),'b', t, qd,'r--','LineWidth',1.5); grid on
legend('q_1','q_{1d}'); ylabel('q_1 (rad)'); title('Tracking Posisi Sendi 1');
subplot(2,1,2)
plot(t, X(:,3),'b', t, qd,'r--','LineWidth',1.5); grid on
legend('q_2','q_{2d}'); ylabel('q_2 (rad)'); xlabel('t (s)');
title('Tracking Posisi Sendi 2');

% Figure 2 : tracking kecepatan
figure('Name','Tracking Kecepatan');
subplot(2,1,1)
plot(t, X(:,2),'b', t, dqd,'r--','LineWidth',1.5); grid on
legend('dq_1','dq_{1d}'); ylabel('dq_1 (rad/s)'); title('Kecepatan Sendi 1');
subplot(2,1,2)
plot(t, X(:,4),'b', t, dqd,'r--','LineWidth',1.5); grid on
legend('dq_2','dq_{2d}'); ylabel('dq_2 (rad/s)'); xlabel('t (s)');
title('Kecepatan Sendi 2');

% Figure 3 : kompensasi gaya gesek
figure('Name','Kompensasi Gesekan');
subplot(2,1,1)
plot(t, FF(:,1),'r', t, FH(:,1),'b--','LineWidth',1.5); grid on
legend('F_1 (nyata)','$\hat{F}_1$ (estimasi)','Interpreter','latex');
ylabel('F_1 (Nm)'); title('Kompensasi Gesekan Sendi 1');
subplot(2,1,2)
plot(t, FF(:,2),'r', t, FH(:,2),'b--','LineWidth',1.5); grid on
legend('F_2 (nyata)','$\hat{F}_2$ (estimasi)','Interpreter','latex');
ylabel('F_2 (Nm)'); xlabel('t (s)'); title('Kompensasi Gesekan Sendi 2');

% Figure 4 : sinyal kontrol torsi
figure('Name','Sinyal Kontrol Torsi');
plot(t, TAU(:,1),'b', t, TAU(:,2),'r','LineWidth',1.5); grid on
legend('\tau_1','\tau_2'); xlabel('t (s)'); ylabel('\tau (Nm)');
title('Sinyal Kontrol Torsi');

%% ===================== Fungsi lokal =====================
function [dX, tau, F, Fhat] = robot_ode(t, X, P)
    % ---- Unpack state ----
    q  = [X(1); X(3)];
    dq = [X(2); X(4)];
    W1 = X(5:9);       % bobot fuzzy sendi 1 (5x1)
    W2 = X(10:14);     % bobot fuzzy sendi 2 (5x1)

    % ---- Trajektori referensi ----
    qd   = [ 0.3*sin(t);  0.3*sin(t)];
    dqd  = [ 0.3*cos(t);  0.3*cos(t)];
    ddqd = [-0.3*sin(t); -0.3*sin(t)];

    % ---- Sliding surface ----
    e    = q - qd;
    de   = dq - dqd;
    s    = de + P.K*e;          % s = dq - dqr
    dqr  = dqd  - P.K*e;
    ddqr = ddqd - P.K*de;

    % ---- Matriks dinamik robot ----
    [D, C, G] = robot_dyn(q, dq, P);

    % ---- Fuzzy compensator (estimasi gesekan) ----
    h1   = fuzzy_basis(dq(1), P.centers, P.sigma);   % 5x1
    h2   = fuzzy_basis(dq(2), P.centers, P.sigma);   % 5x1
    Fhat = [W1.'*h1; W2.'*h2];

    % ---- Hukum kendali ----
    tau = D*ddqr + C*dqr + G + Fhat - P.KD*s - P.Wrob*sign(s);

    % ---- Gesekan nyata pada plant ----
    F = [10*dq(1) + 3*sign(dq(1));
         10*dq(2) + 3*sign(dq(2))];

    % ---- Dinamika plant: D*ddq + C*dq + G + F = tau ----
    ddq = D \ (tau - C*dq - G - F);

    % ---- Hukum adaptif ----
    dW1 = -P.Gamma * s(1) * h1;
    dW2 = -P.Gamma * s(2) * h2;

    % ---- Susun turunan state ----
    dX = zeros(14,1);
    dX(1) = dq(1);  dX(2) = ddq(1);
    dX(3) = dq(2);  dX(4) = ddq(2);
    dX(5:9)   = dW1;
    dX(10:14) = dW2;
end

function [D, C, G] = robot_dyn(q, dq, P)
    m1 = P.m1; m2 = P.m2; l1 = P.l1; l2 = P.l2; g = P.g;
    q1 = q(1); q2 = q(2); dq1 = dq(1); dq2 = dq(2);

    % Matriks inersia
    D11 = (m1+m2)*l1^2 + m2*l2^2 + 2*m2*l1*l2*cos(q2);
    D12 = m2*l2^2 + m2*l1*l2*cos(q2);
    D21 = D12;
    D22 = m2*l2^2;
    D = [D11 D12; D21 D22];

    % Matriks Coriolis & sentrifugal
    C11 = -m2*l1*l2*sin(q2)*dq2;
    C12 = -m2*l1*l2*sin(q2)*(dq1+dq2);
    C21 =  m2*l1*l2*sin(q2)*dq1;
    C22 =  0;
    C = [C11 C12; C21 C22];

    % Vektor gravitasi
    G1 = (m1+m2)*l1*g*cos(q1) + m2*l2*g*cos(q1+q2);
    G2 = m2*l2*g*cos(q1+q2);
    G = [G1; G2];
end

function h = fuzzy_basis(x, centers, sigma)
    % 5 fungsi keanggotaan Gaussian, dinormalisasi (fuzzy basis function)
    mu  = exp(-((x - centers)./sigma).^2);   % 5x1
    den = sum(mu);
    if den < eps, den = eps; end
    h = mu ./ den;
end