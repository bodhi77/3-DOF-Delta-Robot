% =========================================================================
% PLOT RMSE & CHATTERING INDEX VS LOAD (5 CONTROLLERS, FULL LATEX)
% =========================================================================

% 1. Load Data on X-axis (grams)
payload = [0, 500, 1000];

% 2. Dummy RMSE Data (Left Y-axis) - STRICTLY 5 CONTROLLERS
rmse_IT2_NFSC = [1.026747, 0.7362, 1.0391];
rmse_T1_NFSC  = [1.162637, 1.098048, 1.313626];
rmse_IT2_FSC  = [1.064532, 1.131335, 1.424345];
rmse_T1_FSC   = [0.966332, 0.959624, 1.074834];
rmse_SC       = [0.966332, 1.714539, 2.361399];

% 3. Dummy Chattering Data (Right Y-axis) - STRICTLY 5 CONTROLLERS
chat_IT2_NFSC = [0.1415, 0.1601, 0.1834];
chat_T1_NFSC  = [0.1212, 0.1553, 0.1934];
chat_IT2_FSC  = [0.1367, 0.1628, 0.1951];
chat_T1_FSC   = [0.1802, 0.2008, 0.2271];
chat_SC       = [0.1006, 0.1252, 0.1573];

% 4. Figure and Layout Settings
figure('Name', 'Performance Analysis', 'Units', 'normalized', 'Position', [0.1, 0.2, 0.8, 0.55]);
t = tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'normal');

% 5. Create 5 Distinct Colors (Dimmed HSV spectrum)
colors = hsv(5) * 0.8;

% =========================================================================
% TILE 1: RMSE PLOT (LEFT)
% =========================================================================
ax1 = nexttile;
hold(ax1, 'on'); grid(ax1, 'on');

plot(ax1, payload, rmse_IT2_NFSC, 'Color', colors(1,:), 'LineStyle', '-',  'LineWidth', 2.5, 'DisplayName', 'IT2-NFSC');
plot(ax1, payload, rmse_T1_NFSC,  'Color', colors(2,:), 'LineStyle', '--', 'LineWidth', 2.5, 'DisplayName', 'T1-NFSC');
plot(ax1, payload, rmse_IT2_FSC,  'Color', colors(3,:), 'LineStyle', ':',  'LineWidth', 2.5, 'DisplayName', 'IT2-FSC');
plot(ax1, payload, rmse_T1_FSC,   'Color', colors(4,:), 'LineStyle', '-.', 'LineWidth', 2.5, 'DisplayName', 'T1-FSC');
plot(ax1, payload, rmse_SC,       'Color', colors(5,:), 'LineStyle', '-',  'LineWidth', 2.5, 'DisplayName', 'SC');

% Using LaTeX interpreter for all text
xlabel(ax1, '\textbf{Load (g)}', 'Interpreter', 'latex', 'FontSize', 14);
ylabel(ax1, '\textbf{RMSE (degree)}', 'Interpreter', 'latex', 'FontSize', 14);
title(ax1, '\textbf{RMSE Comparison}', 'Interpreter', 'latex', 'FontSize', 16);

xticks(ax1, [0, 500, 1000]);
xlim(ax1, [-100, 1100]);
ax1.FontSize = 13;
ax1.TickLabelInterpreter = 'latex'; 
hold(ax1, 'off');

% =========================================================================
% TILE 2: CHATTERING PLOT (RIGHT)
% =========================================================================
ax2 = nexttile;
hold(ax2, 'on'); grid(ax2, 'on');

% No "DisplayName" needed here to prevent Legend duplication
plot(ax2, payload, chat_IT2_NFSC, 'Color', colors(1,:), 'LineStyle', '-',  'LineWidth', 2.5);
plot(ax2, payload, chat_T1_NFSC,  'Color', colors(2,:), 'LineStyle', '--', 'LineWidth', 2.5);
plot(ax2, payload, chat_IT2_FSC,  'Color', colors(3,:), 'LineStyle', ':',  'LineWidth', 2.5);
plot(ax2, payload, chat_T1_FSC,   'Color', colors(4,:), 'LineStyle', '-.', 'LineWidth', 2.5);
plot(ax2, payload, chat_SC,       'Color', colors(5,:), 'LineStyle', '-',  'LineWidth', 2.5);

% Using LaTeX interpreter for all text
xlabel(ax2, '\textbf{Load (g)}', 'Interpreter', 'latex', 'FontSize', 14);
ylabel(ax2, '\textbf{$\tau_{chatter}$}', 'Interpreter', 'latex', 'FontSize', 16);
title(ax2, '\textbf{$\tau_{chatter}$ Comparison}', 'Interpreter', 'latex', 'FontSize', 16);

xticks(ax2, [0, 500, 1000]);
xlim(ax2, [-100, 1100]);
ax2.FontSize = 13;
ax2.TickLabelInterpreter = 'latex'; 
hold(ax2, 'off');

% =========================================================================
% GLOBAL LEGEND
% =========================================================================
% Calling one shared legend from ax1 and placing it at the bottom (South)
lgd = legend(ax1);
lgd.Layout.Tile = 'South';
lgd.Orientation = 'horizontal';
lgd.NumColumns = 3; 
lgd.FontSize = 13;
lgd.Interpreter = 'latex';
hold off;