% =========================================================================
% SCRIPT DIAGRAM BATANG Chattering Index (0g, 500g, 1000g) - FINAL
% =========================================================================

% 1. Label untuk Sumbu X (Nama-nama Kontroler)
controllers = {
    'T1-NFSC', 'IT2-NFSC', 'SC', ...
    'T1-FSC', 'IT2-FSC', 'PID', 'T1-FPID', ...
    'IT2-FPID', 'SMC', 'T1-FSMC', 'IT2-FSMC'
};

% 2. Matriks Data Chattering Index 
% Format kolom: [0g, 500g, 1000g]
chatter_data = [
    0.4231,  0.5089,  0.4475;   % NeuroMamdaniT1-Syn
    0.5009,  0.5044,  0.4984;   % NeuroIT2F-Syn
    0.3714,  0.3861,  0.3997;   % Synergetic
    0.9668,  1.0370,  0.9572;   % FuzzyT1-Syn
    0.6662,  0.7387,  0.7126;   % FuzzyT2-Syn 
    0.4173,  0.4333,  0.4140;   % PID
    0.3600,  0.3921,  0.4108;   % FuzzyPIDT1
    0.3524,  0.4056,  0.4106;   % FuzzyPIDT2
    1.5430,  1.5284,  1.2251;   % SMC
    1.2847,  1.2051,  1.0234;   % FuzzySMCT1
    1.2171,  1.2760,  0.9863    % FuzzySMCT2
];

% 3. Pengaturan Figur
figure('Name', 'Perbandingan Chattering Index', 'Units', 'normalized', 'Position', [0.1, 0.1, 0.8, 0.6]);

% 4. Membuat Diagram Batang (Grouped)
b = bar(chatter_data, 'grouped');

% 5. Mengatur Warna Batang
b(1).FaceColor = [0.00, 0.45, 0.74]; % Biru untuk 0g
b(2).FaceColor = [0.85, 0.33, 0.10]; % Oranye untuk 500g
b(3).FaceColor = [0.47, 0.67, 0.19]; % Hijau untuk 1000g

% 6. Properti Teks dan Sumbu
set(gca, 'XTick', 1:length(controllers), 'XTickLabel', controllers);
xtickangle(45); % Miringkan teks bawah agar rapi
ylabel('Average Chattering Index (\tau_{chatter})', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Controllers', 'FontSize', 14, 'FontWeight', 'bold');
title('Comparison of Average Chattering Index Across Different Payloads', 'FontSize', 16);
set(gca, 'FontSize', 12, 'LineWidth', 1);

% 7. Grid dan Batas Sumbu Y
grid on;
set(gca, 'GridLineStyle', '--', 'GridAlpha', 0.5);
ylim([0, max(chatter_data(:)) * 1.15]); % Ruang ekstra 15% di atas batang

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