% Script untuk menggambar diagram model simulasi Delta robot (Warna Diperbarui)
clear; clc; close all;

% Setup Figure
fig = figure('Name', 'Simulation Model of Delta Robot', 'Position', [100, 100, 900, 600]);
hold on;
axis off;
axis([0 12 0 10.5]); % Set limit koordinat kanvas

% --- Definisi Warna (Konversi Hex ke RGB MATLAB) ---
color_1   = [150, 186, 109] / 255; % #96BA6D (Hijau Daun)
color_2   = [109, 150, 186] / 255; % #6D96BA (Biru Muda)
color_3   = [186, 109, 150] / 255; % #BA6D96 (Ungu/Magenta)
color_box = [0.40, 0.40, 0.40];     % Abu-abu netral untuk border & garis pembungkus

% --- 1. Menggambar Kotak Putus-putus Kiri (Enclosure) ---
rectangle('Position', [1.5, 0.5, 4.5, 9.5], 'LineStyle', '--', ...
          'EdgeColor', color_box, 'LineWidth', 1.5);

% --- 2. Menggambar Garis Penghubung (Efek Magnifikasi) ---
plot([6.0, 7.5], [10.0, 8.5], '--', 'Color', color_box, 'LineWidth', 1.5);
plot([6.0, 7.5], [0.5, 3.5], '--', 'Color', color_box, 'LineWidth', 1.5);

% --- 3. Memuat dan Menampilkan Gambar Delta Robot (Kanan) ---
img_filename = 'delta_robot.png'; % Ubah ke .png jika file Anda PNG
box_right_pos = [7.5, 3.5, 4.0, 5.0];

% Kotak border gambar
rectangle('Position', box_right_pos, 'EdgeColor', color_box, 'LineWidth', 1.5);

if isfile(img_filename)
    img = imread(img_filename);
    image('XData', [7.52, 11.48], 'YData', [8.48, 3.52], 'CData', img);
else
    text(9.5, 6.0, {'Gambar Delta Robot', ['(' img_filename ')']}, ...
        'HorizontalAlignment', 'center', 'FontSize', 12, 'Color', 'k');
end

% --- 4. Menggambar Blok Motor 1 (Warna 1) ---
draw_motor_block(2.0, 7.5, color_1, '1');

% --- 5. Menggambar Blok Motor 2 (Warna 2) ---
draw_motor_block(2.0, 4.5, color_2, '2');

% --- 6. Menggambar Blok Motor 3 (Warna 3) ---
draw_motor_block(2.0, 1.5, color_3, '3');

% --- 7. Caption Figure ---

hold off;

% =========================================================================
% FUNGSI BANTUAN (HELPER FUNCTIONS)
% =========================================================================

function draw_motor_block(x, y, col, idx)
    % Fungsi untuk menggambar satu set "Motor model" beserta panah I/O-nya
    
    % 1. Gambar Kotak Motor
    width = 2.5; height = 1.2;
    rectangle('Position', [x, y, width, height], 'FaceColor', col, 'EdgeColor', 'none');
    text(x + width/2, y + height/2, 'Motor model', 'Color', 'w', ...
         'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold');
     
    % 2. Input Atas (Tau_L)
    y_top = y + height + 0.6;
    x_in_start = 0.5; x_in_mid = x + 0.5;
    
    plot([x_in_start, x_in_mid], [y_top, y_top], 'Color', col, 'LineWidth', 1.5);
    draw_arrow([x_in_mid, x_in_mid], [y_top, y + height], col);
    
    tex_str = sprintf('$$\\tau_{L%s}$$', idx);
    text(x_in_start + 0.5, y_top + 0.3, tex_str, 'Interpreter', 'latex', ...
         'FontSize', 12, 'Color', 'k');
     
    % 3. Input Samping (Tau)
    y_side = y + 0.3;
    draw_arrow([x_in_start, x], [y_side, y_side], col);
    tex_tau = sprintf('$$\\tau_%s$$', idx);
    text(x_in_start + 0.5, y_side + 0.3, tex_tau, 'Interpreter', 'latex', ...
         'FontSize', 12, 'Color', 'k');
     
    % 4. Output (q, q_dot, q_ddot)
    x_out_start = x + width; x_out_end = x_out_start + 1.0;
    y_out1 = y + 0.9;
    y_out2 = y + 0.6;
    y_out3 = y + 0.3;
    
    draw_arrow([x_out_start, x_out_end], [y_out1, y_out1], col);
    draw_arrow([x_out_start, x_out_end], [y_out2, y_out2], col);
    draw_arrow([x_out_start, x_out_end], [y_out3, y_out3], col);
    
    tex_q1 = sprintf('$$q_%s$$', idx);
    tex_q2 = sprintf('$$\\dot{q}_%s$$', idx);
    tex_q3 = sprintf('$$\\ddot{q}_%s$$', idx);
    
    text(x_out_end + 0.1, y_out1 + 0.15, tex_q1, 'Interpreter', 'latex', 'FontSize', 12);
    text(x_out_end + 0.1, y_out2 + 0.15, tex_q2, 'Interpreter', 'latex', 'FontSize', 12);
    text(x_out_end + 0.1, y_out3 + 0.15, tex_q3, 'Interpreter', 'latex', 'FontSize', 12);
end

function draw_arrow(x_pts, y_pts, col)
    % Fungsi kustom untuk menggambar garis dengan ujung panah (arrowhead)
    plot(x_pts, y_pts, 'Color', col, 'LineWidth', 1.5);
    
    ang = atan2(y_pts(2) - y_pts(1), x_pts(2) - x_pts(1));
    head_len = 0.2;
    head_ang = pi/6;
    
    p1 = [x_pts(2) - head_len*cos(ang - head_ang), y_pts(2) - head_len*sin(ang - head_ang)];
    p2 = [x_pts(2) - head_len*cos(ang + head_ang), y_pts(2) - head_len*sin(ang + head_ang)];
    
    fill([x_pts(2), p1(1), p2(1)], [y_pts(2), p1(2), p2(2)], col, 'EdgeColor', 'none');
end