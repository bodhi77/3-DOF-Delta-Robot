function plot_error_with_stats(filenames, error_headers, dataTime)
    if ischar(filenames)
        filenames = {filenames};
    end
    if ischar(error_headers)
        error_headers = {error_headers};
    end

    figure; 
    hold on;

    total_plots = numel(filenames) * numel(error_headers);
    colors = hsv(total_plots) * 0.8;
    color_idx = 1;

    legend_entries = {}; 

    for f_idx = 1:numel(filenames)
        filename = filenames{f_idx};
        data = readtable(filename);

        disp(['Headers dalam file ', filename, ':']);
        disp(data.Properties.VariableNames);

        for h_idx = 1:numel(error_headers)
            error_header = error_headers{h_idx};

            if ~ismember(error_header, data.Properties.VariableNames)
                warning('Kolom "%s" tidak ditemukan dalam file %s.', error_header, filename);
                continue;
            end

            error_data = data.(error_header);
            valid_idx = ~isnan(error_data);
            error_data = error_data(valid_idx);
            error_data = error_data * (180/pi);   % rad -> deg
            N = length(error_data);
            time = linspace(0, dataTime, N)';

            % Statistik
            mean_abs_error = mean(abs(error_data));     % ✅ Gunakan mean absolut
            std_error = std(error_data);                % ❗ SD tetap dari data asli

            % Warna plot
            plot_color = colors(color_idx, :);

            % Plot error
            plot(time, error_data, 'Color', plot_color, 'LineWidth', 1.5,...
                'DisplayName', sprintf('%s - %s', '\bf', filename, error_header));

            % Garis mean (absolut)
            yline(mean_abs_error, '--', 'Color', plot_color, ...
                'LineWidth', 2, 'DisplayName', sprintf('MeanAbs: %.4f', mean_abs_error));

            % Area ± std deviasi
            fill([time; flipud(time)], ...
                 [repmat(mean_abs_error - std_error, N, 1); flipud(repmat(mean_abs_error + std_error, N, 1))], ...
                 plot_color, 'FaceAlpha', 0.2, 'EdgeColor', 'none', ...
                 'DisplayName', sprintf('StdDev: %.4f', std_error)); 

            legend_entries{end+1} = sprintf('%s - %s', filename, error_header);
            color_idx = color_idx + 1;
        end
    end

    xlabel('Time (s)', 'FontSize', 14);
    ylabel('Error (degree)', 'FontSize', 14);
    title('Error Plot for Multiple Files & Headers', 'FontSize', 16);
    lgd = legend;
    lgd.FontSize = 12;
    grid on;
    hold off;
    xlim([0, dataTime]);
    ylim([-5, 5]);
end

% plot_error_with_stats('ex6.csv', 'e3', 17);
% plot_error_with_stats('0paraPID-.csv', {'e1','e2','e3'}, 17);

plot_error_with_stats('500helixNeuroIT2FS-1.csv', {'e1','e2','e3'}, 7);
% plot_error_with_stats('1000helixNeuroIT2FS-.csv', {'e1','e2','e3'}, 17);

% plot_error_with_stats('0paraNeuroIT2FS-.csv', {'ActualCurr1','ActualCurr2','ActualCurr3'}, 7);
% plot_error_with_stats('0paraSynergetic-.csv', {'ActualCurr1','ActualCurr2','ActualCurr3'}, 7);

% plot_error_with_stats('0paraFuzzySMCT2-.csv', {'e1','e2','e3'}, 17);

% plot_error_with_stats('0paraSynergetic-.csv', {'e1','e2','e3'}, 17);

% plot_error_with_stats({'PID1.csv', 'FST2-2.csv'}, {'e1','e2','e3'}, 17);

% plot_error_with_stats({'ex6.csv', 'ex7.csv'}, {'e1', 'e3'}, 17);