% =========================================================================
% Script Lengkap: Robot Delta Kinematics dengan 3 Bola Interseksi
% =========================================================================
clear; clc; close all;
figure('Color', 'w', 'Name', 'Final Delta Robot Kinematics', 'Position', [50, 50, 950, 750]);
hold on; axis equal; axis off;
view([-25, 30]); 
camzoom(1.35);   

%% 0. DEFINISI WARNA
c_base = [123, 224, 72] / 255;  % #7BE048 (Hijau)
c_plat = [72, 123, 224] / 255;  % #487BE0 (Biru)

%% 1. PARAMETER
R_base_vert = 3.5; R_plat_vert = 4.5; L_upper = 4.375;
Z_plat = -7.5; Z_drop = -3.0; 
P0 = [0, 0, Z_plat]; % Platform tepat di tengah
sudut_vert = [30, 150, 270]; 

%% 2. KOORDINAT
B_vert = zeros(3, 3); P_vert = zeros(3, 3); B_mid = zeros(3, 3); P_mid = zeros(3, 3); E = zeros(3, 3);
for i = 1:3
    B_vert(i, :) = [R_base_vert*cosd(sudut_vert(i)), R_base_vert*sind(sudut_vert(i)), 0];
    P_vert(i, :) = P0 + [R_plat_vert*cosd(sudut_vert(i)), R_plat_vert*sind(sudut_vert(i)), 0];
end
B_mid(1, :) = (B_vert(3, :) + B_vert(1, :)) / 2;
B_mid(2, :) = (B_vert(1, :) + B_vert(2, :)) / 2;
B_mid(3, :) = (B_vert(2, :) + B_vert(3, :)) / 2;
P_mid(1, :) = (P_vert(3, :) + P_vert(1, :)) / 2;
P_mid(2, :) = (P_vert(1, :) + P_vert(2, :)) / 2;
P_mid(3, :) = (P_vert(2, :) + P_vert(3, :)) / 2;
sudut_joint = [330, 90, 210];
for i=1:3
    E(i, :) = B_mid(i, :) + [L_upper*cosd(sudut_joint(i)), L_upper*sind(sudut_joint(i)), Z_drop];
end
R_hex = (R_base_vert * cosd(60)) / cosd(30); 
Hex_base = zeros(6, 3);
for i = 1:6, Hex_base(i, :) = [R_hex*cosd((i-1)*60), R_hex*sind((i-1)*60), 0]; end

%% 3. BOLA & FISIK ROBOT
R_sph = norm(E(1,:) - P0); [sx, sy, sz] = sphere(40);
w_c = [1.0, 0.9, 0.2; 0.3, 0.6, 1.0; 1.0, 0.5, 0.8];
for i = 1:3
    % Permukaan Bola Transparan
    surf(sx*R_sph + E(i,1), sy*R_sph + E(i,2), sz*R_sph + E(i,3), 'FaceColor', w_c(i,:), 'FaceAlpha', 0.1, 'EdgeColor', 'none');
    
    % Lingkaran Tegas (Great Circles) yang memotong P0
    v1 = [cosd(sudut_joint(i)), sind(sudut_joint(i)), 0];
    v2 = [0, 0, 1];
    theta_circ = linspace(0, 2*pi, 100);
    Xc = E(i,1) + R_sph * (v1(1)*cos(theta_circ) + v2(1)*sin(theta_circ));
    Yc = E(i,2) + R_sph * (v1(2)*cos(theta_circ) + v2(2)*sin(theta_circ));
    Zc = E(i,3) + R_sph * (v1(3)*cos(theta_circ) + v2(3)*sin(theta_circ));
    plot3(Xc, Yc, Zc, 'Color', w_c(i,:), 'LineWidth', 2.5);
end
fill3(Hex_base(:,1), Hex_base(:,2), Hex_base(:,3), c_base, 'FaceAlpha', 0.5, 'EdgeColor', [0.4 0.4 0.4]);

% Platform Bawah (Continuous Poly)
R_joint = R_plat_vert * cosd(60); R_circ = R_joint / 3; w_s = 0.5; t_i = asind((w_s/2)/R_circ);
Xp=[]; Yp=[]; 
s_u = [330, 90, 210]; % Sinkronkan urutan sudut spoke visual dengan posisi P_mid
for i=1:3
    t_a = linspace((s_u(i)-120)+t_i, s_u(i)-t_i, 20);
    Xp=[Xp, P0(1)+R_circ*cosd(t_a), P0(1)+R_joint*cosd(s_u(i))+(w_s/2)*cosd(s_u(i)-90), P0(1)+R_joint*cosd(s_u(i))+(w_s/2)*cosd(s_u(i)+90)];
    Yp=[Yp, P0(2)+R_circ*sind(t_a), P0(2)+R_joint*sind(s_u(i))+(w_s/2)*sind(s_u(i)-90), P0(2)+R_joint*sind(s_u(i))+(w_s/2)*sind(s_u(i)+90)];
end
fill3(Xp, Yp, P0(3)*ones(size(Xp)), c_plat, 'FaceAlpha', 0.8, 'EdgeColor', [0.4 0.4 0.4]);

for i=1:3, plot3([B_mid(i,1), E(i,1), P_mid(i,1)], [B_mid(i,2), E(i,2), P_mid(i,2)], [B_mid(i,3), E(i,3), P_mid(i,3)], 'k-', 'LineWidth', 2); end

% Titik E0
plot3(P0(1), P0(2), P0(3), 'r.', 'MarkerSize', 20);
text(P0(1)+0.5, P0(2)-0.5, P0(3), 'E_0(x_0; y_0; z_0)', 'FontSize', 12, 'FontWeight', 'bold');

%% 4. TAMBAHAN ANOTASI F, J', DAN PANAH MERAH
% Label F1, F2, F3 di Base Atas
text(B_mid(1,1)+0.4, B_mid(1,2)+0.4, B_mid(1,3), 'F_1', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');
text(B_mid(2,1)-1.0, B_mid(2,2)+0.4, B_mid(2,3), 'F_2', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');
text(B_mid(3,1)+0.6, B_mid(3,2)-0.6, B_mid(3,3), 'F_3', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');

% Label J'_1, J'_2, J'_3 & Panah Merah di Area Siku
for i = 1:3
    u = 1.0 * cosd(sudut_joint(i));
    v = 1.0 * sind(sudut_joint(i));
    quiver3(E(i,1), E(i,2), E(i,3), u, v, 0, 'Color', 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.8);
    text(E(i,1)+u*1.2, E(i,2)+v*1.2, E(i,3), ['J''_', num2str(i)], 'FontSize', 14, 'FontWeight', 'bold', 'Color', w_c(i,:));
end

hold off;