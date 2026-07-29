%% =====================================================================
%  TUGAS 1 : INVERTED PENDULUM  -  FUZZY T-S + PDC + LMI (Lyapunov)
%  Kompatibel MATLAB R2024b.
%  Penyelesaian LMI memakai LMI Lab (Robust Control Toolbox: feasp)
%  atau YALMIP bila terpasang. Dipilih otomatis.
%% =====================================================================
clear; clc; close all;

%% ----------------- Parameter sistem -----------------
g    = 9.81;          % gravitasi (m/s^2)
m    = 2.0;           % massa pendulum (kg)
M    = 8.0;           % massa kereta (kg)
l    = 0.5;           % setengah panjang pendulum (m)
a    = 1/(m+M);       % a = 1/(m+M)
beta = cos(88*pi/180);% beta = cos(88 derajat)

% Penyebut model linear
den1 = 4*l/3 - a*m*l;
den2 = 4*l/3 - a*m*l*beta^2;

% ----------------- Matriks model Fuzzy T-S -----------------
A1 = [0          1;
      g/den1     0];
B1 = [0;
      -a/den1];

A2 = [0                 1;
      2*g/(pi*den2)     0];
B2 = [0;
      -a*beta/den2];

fprintf('--- Matriks model T-S ---\n');
disp('A1 ='); disp(A1); disp('B1 ='); disp(B1);
disp('A2 ='); disp(A2); disp('B2 ='); disp(B2);

%% ----------------- Penyelesaian LMI -----------------
% Variabel : Q (2x2 simetris, Q>0), V1 (1x2), V2 (1x2)
% LMI :
%  1) Q*A1' + A1*Q + V1'*B1' + B1*V1 < 0
%  2) Q*A2' + A2*Q + V2'*B2' + B2*V2 < 0
%  3) Q*A1'+A1*Q + Q*A2'+A2*Q + V2'*B1'+B1*V2 + V1'*B2'+B2*V1 < 0
%  4) Q > 0
% Gain : K1 = V1*inv(Q), K2 = V2*inv(Q)

if exist('feasp','file') == 2
    % ---------- LMI Lab (Robust Control Toolbox) ----------
    setlmis([]);
    Q  = lmivar(1, [2 1]);   % simetris 2x2
    V1 = lmivar(2, [1 2]);   % penuh 1x2
    V2 = lmivar(2, [1 2]);   % penuh 1x2

    % LMI 1
    lmiterm([1 1 1 Q],  A1, 1, 's');   % A1*Q + Q*A1'
    lmiterm([1 1 1 V1], B1, 1, 's');   % B1*V1 + V1'*B1'
    % LMI 2
    lmiterm([2 1 1 Q],  A2, 1, 's');
    lmiterm([2 1 1 V2], B2, 1, 's');
    % LMI 3
    lmiterm([3 1 1 Q],  A1, 1, 's');
    lmiterm([3 1 1 Q],  A2, 1, 's');
    lmiterm([3 1 1 V2], B1, 1, 's');   % B1*V2 + V2'*B1'
    lmiterm([3 1 1 V1], B2, 1, 's');   % B2*V1 + V1'*B2'
    % LMI 4 : Q > 0  (0 < Q)
    lmiterm([-4 1 1 Q], 1, 1);

    lmis = getlmis;
    [tmin, xfeas] = feasp(lmis);
    if tmin > 0
        warning('LMI mungkin tidak feasible (tmin = %.4g).', tmin);
    end
    Qs  = dec2mat(lmis, xfeas, Q);
    V1s = dec2mat(lmis, xfeas, V1);
    V2s = dec2mat(lmis, xfeas, V2);

elseif exist('sdpvar','file') == 2
    % ---------- YALMIP ----------
    Q  = sdpvar(2,2,'symmetric');
    V1 = sdpvar(1,2,'full');
    V2 = sdpvar(1,2,'full');
    eps0 = 1e-6;
    Con = [ Q >= eps0*eye(2) ];
    Con = [ Con, A1*Q+Q*A1'+B1*V1+V1'*B1' <= -eps0*eye(2) ];
    Con = [ Con, A2*Q+Q*A2'+B2*V2+V2'*B2' <= -eps0*eye(2) ];
    Con = [ Con, A1*Q+Q*A1'+A2*Q+Q*A2' ...
                 + B1*V2+V2'*B1' + B2*V1+V1'*B2' <= 0 ];
    optimize(Con, [], sdpsettings('verbose',0));
    Qs  = value(Q);
    V1s = value(V1);
    V2s = value(V2);
else
    error(['Tidak ada penyelesai LMI. Pasang Robust Control Toolbox ' ...
           '(fungsi feasp) atau YALMIP.']);
end

% ----------------- Gain kontroler -----------------
K1 = V1s / Qs;
K2 = V2s / Qs;

fprintf('\n--- Hasil LMI ---\n');
disp('Q ='); disp(Qs);
fprintf('K1 = [% .4f  % .4f]\n', K1(1), K1(2));
fprintf('K2 = [% .4f  % .4f]\n', K2(1), K2(2));

%% ----------------- Simulasi plant non-linear -----------------
x0    = [pi/3; 0];        % [theta(0); dtheta(0)]
tspan = [0 5];            % 5 detik
[t, X] = ode45(@(t,x) pend_ode(t,x,K1,K2,g,m,l,a), tspan, x0);

% Hitung ulang sinyal kendali u untuk plotting
u = zeros(numel(t),1);
for k = 1:numel(t)
    [~, u(k)] = pend_ode(t(k), X(k,:).', K1, K2, g, m, l, a);
end

%% ----------------- Plot -----------------
% Figure 1 : posisi & kecepatan sudut
figure('Name','State Pendulum');
plot(t, X(:,1), 'b', t, X(:,2), 'r', 'LineWidth', 1.5); grid on
legend('\theta (rad)', 'd\theta/dt (rad/s)');
xlabel('Waktu (s)'); ylabel('State');
title('Posisi Sudut \theta dan Kecepatan Sudut \theta'' ');

% Figure 2 : sinyal kendali
figure('Name','Sinyal Kendali');
plot(t, u, 'k', 'LineWidth', 1.5); grid on
xlabel('Waktu (s)'); ylabel('u(t)');
title('Sinyal Kendali / Control Input u(t)');

%% ===================== Fungsi lokal =====================
function [dx, u] = pend_ode(~, x, K1, K2, g, m, l, a)
    x1 = x(1);   % theta
    x2 = x(2);   % dtheta

    % ---- Membership function segitiga (w1 + w2 = 1) ----
    % w2 naik linear dari 0 (x1=0) ke 1 (|x1|=pi/2); w1 = 1 - w2
    z  = min(abs(x1)/(pi/2), 1);   % saturasi pada |x1| < pi/2
    w2 = z;
    w1 = 1 - z;

    % ---- Kontroler PDC ----
    u = (w1*K1 + w2*K2) * x;       % karena w1 + w2 = 1

    % ---- Dinamika non-linear inverted pendulum ----
    num = g*sin(x1) - a*m*l*x2^2*sin(2*x1)/2 - a*cos(x1)*u;
    den = 4*l/3 - a*m*l*cos(x1)^2;
    ddx = num/den;

    dx = [x2; ddx];
end