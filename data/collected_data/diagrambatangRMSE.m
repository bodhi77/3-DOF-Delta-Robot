% =========================================================================
% SCRIPT DIAGRAM BATANG RMSE (0g, 500g, 1000g) - FINAL & LENGKAP
% =========================================================================

% 1. Label untuk Sumbu X (Nama-nama Kontroler)
controllers = {
    'NeuroMamdaniT1-Syn', 'NeuroIT2F-Syn', 'Synergetic', ...
    'FuzzyT1-Syn', 'FuzzyT2-Syn', 'PID', 'FuzzyPIDT1', ...
    'FuzzyPIDT2', 'SMC', 'FuzzySMCT1', 'FuzzySMCT2'
};

% 2. Matriks Data RMSE Terverifikasi (Kompilasi 3 File)
% Format kolom: [0g, 500g, 1000g]
rmse_data = [
    0.804823,  0.735261,  0.993138;   % NeuroMamdaniT1-Syn
    0.600285,  0.746525,  0.937319;   % NeuroIT2F-Syn
    1.125263,  1.176312,  1.423891;   % Synergetic
    0.989343,  0.980601,  1.007742;   % FuzzyT1-Syn
    1.139839,  1.248800,  1.303523;   % FuzzyT2-Syn (SEKARANG SUDAH TERISI)
    0.754251,  0.971036,  1.272970;   % PID
    1.458312,  1.623491,  2.062138;   % FuzzyPIDT1
    1.295005,  1.457952,  1.961870;   % FuzzyPIDT2
    0.595680,  1.148718,  2.142493;   % SMC
    1.175739,  1.471632,  2.108365;   % FuzzySMCT1
    1.193433,  1.345150,  2.414847    % FuzzySMCT2
];

% 3. Pengaturan Figur
figure('Name', 'Perbandingan RMSE', 'Units', 'normalized', 'Position', [0.1, 0.1, 0.8, 0.6]);

% 4. Membuat Diagram Batang (Grouped)
b = bar(rmse_data, 'grouped');

% 5. Mengatur Warna Batang (Standar Visibilitas Jurnal)
b(1).FaceColor = [0.00, 0.45, 0.74]; % Biru untuk 0g
b(2).FaceColor = [0.85, 0.33, 0.10]; % Oranye untuk 500g
b(3).FaceColor = [0.47, 0.67, 0.19]; % Hijau untuk 1000g

% 6. Properti Teks dan Sumbu
set(gca, 'XTick', 1:length(controllers), 'XTickLabel', controllers);
xtickangle(45); % Miringkan teks agar tidak bertumpuk
ylabel('Average RMSE (Degree)', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Controllers', 'FontSize', 14, 'FontWeight', 'bold');
title('Comparison of Average RMSE Across Different Payloads', 'FontSize', 16);
set(gca, 'FontSize', 12, 'LineWidth', 1);

% 7. Grid dan Batas Sumbu Y
grid on;
set(gca, 'GridLineStyle', '--', 'GridAlpha', 0.5);
ylim([0, max(rmse_data(:)) * 1.15]); % Memberi ruang ekstra 15% di bagian atas

% 8. Membuat Legend
lgd = legend('0g Payload', '500g Payload', '1000g Payload', 'Location', 'northwest');
lgd.FontSize = 12;

% =========================================================================
% Tampilkan Nilai Angka di Atas Batang
% =========================================================================
for i = 1:length(b)
    xtips = b(i).XEndPoints;
    ytips = b(i).YEndPoints;
    labels = string(round(b(i).YData, 2)); % Pembulatan 2 angka di belakang koma
    text(xtips, ytips, labels, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', 'FontSize', 10, 'Rotation', 90);
end