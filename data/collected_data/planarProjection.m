% =========================================================================
% Script MATLAB: Skematik Robot Delta (Sumbu Original RGB & Pelat Custom)
% =========================================================================

clear; clc; close all;

figure('Color', 'w', 'Name', 'Delta Robot Kinematics - Original Axes', 'Position', [100, 100, 850, 650]);
hold on; axis equal; axis off;
view([-25, 35]); % Sudut pandang 3D
camzoom(1.6);   % Ukuran gambar keseluruhan diperbesar

%% 0. DEFINISI WARNA PELAT FISIK
c_base = [123, 224, 72] / 255;  % #7BE048 (Hijau Terang) untuk segi enam
c_plat = [72, 123, 224] / 255;  % #487BE0 (Biru) untuk platform 

%% 1. PARAMETER GEOMETRI 
R_base_vert = 3.5;    
R_plat_vert = 4.5;    
L_upper     = 4.375;  

Z_plat = -7.5;        
Z_drop = -3.0;        
P0 = [1.5, -1.0, Z_plat]; 

sudut_vert = [30, 150, 270]; 

%% 2. MENGHITUNG KOORDINAT TITIK
B_vert = zeros(3, 3); 
P_vert = zeros(3, 3);
B_mid = zeros(3, 3);
P_mid = zeros(3, 3);
E = zeros(3, 3);

for i = 1:3
    B_vert(i, :) = [R_base_vert*cosd(sudut_vert(i)), R_base_vert*sind(sudut_vert(i)), 0];
    P_vert(i, :) = P0 + [R_plat_vert*cosd(sudut_vert(i)), R_plat_vert*sind(sudut_vert(i)), 0];
end

B_mid(1, :) = (B_vert(3, :) + B_vert(1, :)) / 2; % Theta 1 (330 deg)
B_mid(2, :) = (B_vert(1, :) + B_vert(2, :)) / 2; % Theta 2 (90 deg)
B_mid(3, :) = (B_vert(2, :) + B_vert(3, :)) / 2; % Theta 3 (210 deg)

P_mid(1, :) = (P_vert(3, :) + P_vert(1, :)) / 2;
P_mid(2, :) = (P_vert(1, :) + P_vert(2, :)) / 2;
P_mid(3, :) = (P_vert(2, :) + P_vert(3, :)) / 2;

sudut_joint = [330, 90, 210];
for i=1:3
    E(i, :) = B_mid(i, :) + [L_upper*cosd(sudut_joint(i)), L_upper*sind(sudut_joint(i)), Z_drop];
end

% KOORDINAT SEGI ENAM ROTASI & MEPET SISI
hex_sudut = 0:60:300; 
R_hex = (R_base_vert * cosd(60)) / cosd(30); 

Hex_base = zeros(6, 3);
for i = 1:6
    Hex_base(i, :) = [R_hex*cosd(hex_sudut(i)), R_hex*sind(hex_sudut(i)), 0];
end

%% 3. MENGGAMBAR FISIK ROBOT
warna_garis_frame = [0.4 0.4 0.4];

% Gambar Basis: Segi Enam (Hijau Terang)
fill3(Hex_base(:,1), Hex_base(:,2), Hex_base(:,3), c_base, 'FaceAlpha', 0.5, 'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 1.5);

% Segitiga Basis & Platform (Dashed)
plot3([B_vert(:,1); B_vert(1,1)], [B_vert(:,2); B_vert(1,2)], [B_vert(:,3); B_vert(1,3)], ...
      '--', 'Color', warna_garis_frame, 'LineWidth', 1.5);
plot3([P_vert(:,1); P_vert(1,1)], [P_vert(:,2); P_vert(1,2)], [P_vert(:,3); P_vert(1,3)], ...
      '--', 'Color', warna_garis_frame, 'LineWidth', 1.5);

% ALGORITMA CONTINUOUS PERIMETER 2D PLATFORM BAWAH
R_joint = R_plat_vert * cosd(60);   
R_circ = R_joint / 3; 
w_spoke = 0.5;        
theta_int = asind((w_spoke/2) / R_circ); 

X_poly = []; Y_poly = [];
sudut_spoke_urutan = [90, 210, 330]; 

for i = 1:3
    alpha = sudut_spoke_urutan(i);
    if i == 1
        alpha_prev = sudut_spoke_urutan(3) - 360; 
    else
        alpha_prev = sudut_spoke_urutan(i-1);
    end
    
    t_arc = linspace(alpha_prev + theta_int, alpha - theta_int, 30);
    X_poly = [X_poly, P0(1) + R_circ*cosd(t_arc)];
    Y_poly = [Y_poly, P0(2) + R_circ*sind(t_arc)];
    
    tip_R_x = P0(1) + R_joint*cosd(alpha) + (w_spoke/2)*cosd(alpha-90);
    tip_R_y = P0(2) + R_joint*sind(alpha) + (w_spoke/2)*sind(alpha-90);
    tip_L_x = P0(1) + R_joint*cosd(alpha) + (w_spoke/2)*cosd(alpha+90);
    tip_L_y = P0(2) + R_joint*sind(alpha) + (w_spoke/2)*sind(alpha+90);
    
    X_poly = [X_poly, tip_R_x, tip_L_x];
    Y_poly = [Y_poly, tip_R_y, tip_L_y];
end
Z_poly = P0(3) * ones(size(X_poly));

% Pelat Bawah (Biru)
fill3(X_poly, Y_poly, Z_poly, c_plat, 'FaceAlpha', 0.5, 'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 1.5);

% Garis konstruksi vertikal pusat O ke P0
plot3([0, P0(1)], [0, P0(2)], [0, P0(3)], 'k-', 'LineWidth', 0.5);
plot3(P0(1), P0(2), P0(3), 'r.', 'MarkerSize', 12);

% Lengan Robot
for i = 1:3
    plot3([B_mid(i,1), E(i,1)], [B_mid(i,2), E(i,2)], [B_mid(i,3), E(i,3)], 'k', 'LineWidth', 2.5);
    plot3([E(i,1), P_mid(i,1)], [E(i,2), P_mid(i,2)], [E(i,3), P_mid(i,3)], 'k', 'LineWidth', 2);
    plot3(B_mid(i,1), B_mid(i,2), B_mid(i,3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
    plot3(E(i,1), E(i,2), E(i,3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 4);
    plot3(P_mid(i,1), P_mid(i,2), P_mid(i,3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
end

%% 4. PLOT SUMBU KOORDINAT (KEMBALI KE WARNA ORIGINAL)
O = [0, 0, 0];
faktor_panjang = 0.80; 
lw_vec = 1.2; 

vektor_Yp_asli = B_vert(3, :) - B_mid(2, :);
vektor_Yp_pendek = vektor_Yp_asli * faktor_panjang;
L_axis = norm(vektor_Yp_pendek); 

% =======================================================
% SUMBU Z (Biru Solid Original)
% =======================================================
L_z_half = 4.5; 
warna_Z = [0.2, 0.5, 0.9];
quiver3(0, 0, L_z_half, 0, 0, -2*L_z_half, 0, 'Color', warna_Z, ...
        'LineWidth', lw_vec, 'MaxHeadSize', 0.2, 'LineStyle', '-', 'Marker', '.');
text(0, 0, -L_z_half - 0.6, 'Z', 'Color', warna_Z, 'FontSize', 16, 'FontWeight', 'bold');

% =======================================================
% SUMBU Y' (Hijau Solid Original)
% =======================================================
titik_ujung_Yp = B_mid(2, :) + vektor_Yp_pendek;
warna_Yp_solid = [0, 0.6, 0.2]; 
quiver3(B_mid(2,1), B_mid(2,2), B_mid(2,3), ...
        vektor_Yp_pendek(1), vektor_Yp_pendek(2), vektor_Yp_pendek(3), 0, ...
        'Color', warna_Yp_solid, 'LineWidth', lw_vec, 'MaxHeadSize', 0.3, 'LineStyle', '-', 'Marker', '.');
text(titik_ujung_Yp(1), titik_ujung_Yp(2) - 0.4, titik_ujung_Yp(3), 'Y''', ...
     'Color', warna_Yp_solid, 'FontSize', 16, 'FontWeight', 'bold');

% =======================================================
% SUMBU Y (Hijau Transparan/Pudar Original)
% =======================================================
vektor_Y_asli = B_vert(2, :) - B_mid(1, :);
vektor_Y_pendek = vektor_Y_asli * faktor_panjang;
titik_ujung_Y = B_mid(1, :) + vektor_Y_pendek;
warna_Y_transparan = [0.6, 0.85, 0.6]; 
warna_Y_teks = [0.4, 0.7, 0.4]; 
quiver3(B_mid(1,1), B_mid(1,2), B_mid(1,3), ...
        vektor_Y_pendek(1), vektor_Y_pendek(2), vektor_Y_pendek(3), 0, ...
        'Color', warna_Y_transparan, 'LineWidth', lw_vec, 'MaxHeadSize', 0.3, 'LineStyle', '-', 'Marker', '.');
text(titik_ujung_Y(1) - 0.5, titik_ujung_Y(2) + 0.5, titik_ujung_Y(3), 'Y', ...
     'Color', warna_Y_teks, 'FontSize', 16, 'FontWeight', 'bold');

% =======================================================
% SUMBU X' (Merah Solid Original)
% =======================================================
vXp_dir = [cosd(180), sind(180), 0];
warna_Xp_solid = [1, 0, 0]; 
quiver3(O(1), O(2), O(3), ...
        L_axis*vXp_dir(1), L_axis*vXp_dir(2), 0, 0, ...
        'Color', warna_Xp_solid, 'LineWidth', lw_vec, 'MaxHeadSize', 0.3, 'LineStyle', '-', 'Marker', '.');
text(L_axis*vXp_dir(1) - 0.6, L_axis*vXp_dir(2), 0, 'X''', ...
     'Color', warna_Xp_solid, 'FontSize', 16, 'FontWeight', 'bold');

% =======================================================
% SUMBU X (Merah Transparan/Pudar Original)
% =======================================================
vX_dir = [cosd(60), sind(60), 0];
warna_X_transparan = [1, 0.6, 0.6]; 
warna_X_teks = [0.8, 0.4, 0.4]; 
quiver3(O(1), O(2), O(3), ...
        L_axis*vX_dir(1), L_axis*vX_dir(2), 0, 0, ...
        'Color', warna_X_transparan, 'LineWidth', lw_vec, 'MaxHeadSize', 0.3, 'LineStyle', '-', 'Marker', '.');
text(L_axis*vX_dir(1) + 0.4, L_axis*vX_dir(2) + 0.4, 0, 'X', ...
     'Color', warna_X_teks, 'FontSize', 16, 'FontWeight', 'bold');

% Label Pusat Koordinat O
text(0.3, 0.3, -0.3, 'O', 'Color', [0.4 0.4 0.4], 'FontSize', 14, 'FontWeight', 'bold');

%% 5. BUSUR PANAH ROTASI 120 DERAJAT (Merah Original)
t_arc = linspace(60, 180, 50);
arc_r = 1.5; 
plot3(arc_r*cosd(t_arc), arc_r*sind(t_arc), zeros(1,50), 'r-', 'LineWidth', 1.5);

quiver3(arc_r*cosd(170), arc_r*sind(170), 0, ...
        arc_r*(cosd(180)-cosd(170)), arc_r*(sind(180)-sind(170)), 0, ...
        0, 'Color', 'r', 'MaxHeadSize', 4, 'LineWidth', 1.0);
text(arc_r*cosd(120)*1.3, arc_r*sind(120)*1.3, 0.5, '120^o', 'Color', [0.7 0 0], 'FontSize', 16, 'FontWeight', 'bold');

%% 6. ANOTASI LABEL THETA
text(B_mid(1,1)+0.4, B_mid(1,2)-0.4, B_mid(1,3), '\theta_1', 'FontSize', 16, 'FontWeight', 'bold', 'Color', 'k');
text(B_mid(2,1)+0.4, B_mid(2,2)+0.4, B_mid(2,3), '\theta_2', 'FontSize', 16, 'FontWeight', 'bold', 'Color', 'k');
text(B_mid(3,1)-0.8, B_mid(3,2)-0.4, B_mid(3,3), '\theta_3', 'FontSize', 16, 'FontWeight', 'bold', 'Color', 'k');

hold off;