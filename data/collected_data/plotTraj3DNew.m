function plot_multiple_trajectories_from_csv(filenames, headers)
    % Memastikan jumlah file dan header sesuai
    if length(filenames) ~= length(headers)
        error('Jumlah file dan header harus sama.');
    end
    
    num_files = length(filenames);
    colors = distinguishable_colors(num_files); 
    
    % =========================================================================
    % KUMPULAN GAYA KOMBINASI UNIK (LINESTYLE + MARKER)
    % =========================================================================
    styles_pool =  {'-',  ':',  '-.', '-',  '-',  '-',  ':',  '-.', '-',  ':',  '-.'};
    markers_pool = {'none','none','none','*',  '+',  '>',  'x',  's',  '^',  'D',  '*'};
    % =========================================================================
    
    figure('Name', 'Trajectory Analysis', 'Units', 'normalized', 'Position', [0.1, 0.1, 0.8, 0.7]);
    t = tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    
    % --- TILE 1: ISOMETRIC ---
    ax1 = nexttile(t); hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal'); view(ax1, 3);
    xlabel(ax1, 'X (mm)', 'FontSize', 14); ylabel(ax1, 'Y (mm)', 'FontSize', 14); zlabel(ax1, 'Z (mm)', 'FontSize', 14);
    title(ax1, 'Isometric View', 'FontSize', 16); ax1.FontSize = 12;
    
    % --- TILE 2: TOP VIEW (XY) ---
    ax2 = nexttile(t); hold(ax2, 'on'); grid(ax2, 'on'); axis(ax2, 'equal'); view(ax2, 2);
    xlabel(ax2, 'X (mm)', 'FontSize', 14); ylabel(ax2, 'Y (mm)', 'FontSize', 14);
    title(ax2, 'Top View (XY)', 'FontSize', 16); ax2.FontSize = 12;
    
    h_lines = gobjects(0);
    h_labels = {};
    method_plotted = false(1, num_files);
    ref_plotted = false;
    
    for f = 1:num_files
        data = readtable(filenames{f});
        
        [~, method_name, ~] = fileparts(filenames{f});
        method_name = strrep(method_name, '0helix', ''); 
        method_name = strrep(method_name, '-1', '');
        method_name = strrep(method_name, '-2', '');
        
        if iscell(headers{f}{1})
            num_trajectories = length(headers{f});
        else
            num_trajectories = 1;
            headers{f} = {headers{f}}; 
        end
        
        idx_gaya = mod(f-1, length(styles_pool)) + 1;
        current_style = styles_pool{idx_gaya};
        current_marker = markers_pool{idx_gaya};
        
        for t_idx = 1:num_trajectories
            for i = 1:3
                if ~ismember(headers{f}{t_idx}{i}, data.Properties.VariableNames)
                    error('Header "%s" tidak ditemukan dalam file %s.', headers{f}{t_idx}{i}, filenames{f});
                end
            end
            
            X = data.(headers{f}{t_idx}{1});
            Y = data.(headers{f}{t_idx}{2});
            Z = data.(headers{f}{t_idx}{3});
            
            % Kerapatan marker dikurangi menjadi 15 titik agar plot lebih bersih
            marker_spacing = round(linspace(1, length(X), 100));
            
            traj_name = char(headers{f}{t_idx}{1});
            is_reference = contains(traj_name, '0') || ...
                           contains(traj_name, 'ref', 'IgnoreCase', true) || ...
                           contains(traj_name, 'Set', 'IgnoreCase', true);
            
            if is_reference
                p_ref = plot3(ax1, X, Y, Z, 'k-', 'LineWidth', 2.5); 
                plot(ax2, X, Y, 'k-', 'LineWidth', 2.5);             
                
                if ~ref_plotted
                    h_lines = [p_ref, h_lines]; 
                    h_labels = [{'Reference'}, h_labels];
                    ref_plotted = true;
                end
            else
                % Menerapkan MarkerSize kecil (nilai 4) agar tidak dominan dan mengganggu
                p_method = plot3(ax1, X, Y, Z, 'Color', colors(f, :), 'LineWidth', 1.5, ...
                                 'LineStyle', current_style, 'Marker', current_marker, ...
                                 'MarkerSize', 4, 'MarkerIndices', marker_spacing); 
                             
                plot(ax2, X, Y, 'Color', colors(f, :), 'LineWidth', 1.5, ...
                     'LineStyle', current_style, 'Marker', current_marker, ...
                     'MarkerSize', 4, 'MarkerIndices', marker_spacing);             
                
                if ~method_plotted(f)
                    h_lines(end+1) = p_method;
                    h_labels{end+1} = method_name;
                    method_plotted(f) = true;
                end
            end
        end
    end
    
    hold(ax1, 'off'); hold(ax2, 'off');
    
    if ~isempty(h_lines)
        lgd = legend(h_lines, h_labels);
        lgd.Layout.Tile = 'South';
        lgd.Orientation = 'horizontal';
        lgd.NumColumns = 4; 
        lgd.FontSize = 12;
    end
end

function colors = distinguishable_colors(n)
    colors = hsv(n) * 0.8;
end
% =========================================================================
% BLOK PEMANGGILAN FUNGSI (EKSEKUSI PLOT)
% =========================================================================

% 1. Daftar file CSV yang akan dibandingkan
files_to_plot = {
    '0helixNeuroMamdaniT1-1.csv', ...
    '0helixNeuroIT2FS-1.csv', ...
    '0helixSynergetic-1.csv', ...
    '0helixFuzzySynergeticT1-1.csv', ...
    '0helixFuzzySynergeticT2-1.csv', ...
    '0helixPID-1.csv', ...
    '0helixFuzzyPIDT1-1.csv', ...
    '0helixFuzzyPIDT2-1.csv', ...
    '0helixSMC-1.csv', ...
    '0helixFuzzySMCT1-1.csv', ...
    '0helixFuzzySMCT2-1.csv'
};

% 2. Daftar header untuk masing-masing file
% File pertama memanggil lintasan referensi (X0, Y0, Z0) beserta aktualnya.
% File sisanya cukup memanggil lintasan aktualnya saja.
headers_to_plot = { 
    { {'X0', 'Y0', 'Z0'}, {'Actual_X', 'Actual_Y', 'Actual_Z'} }, % File 1 (Termasuk Referensi)
    { {'Actual_X', 'Actual_Y', 'Actual_Z'} },                     % File 2
    { {'Actual_X', 'Actual_Y', 'Actual_Z'} },                     % File 3
    { {'Actual_X', 'Actual_Y', 'Actual_Z'} },                     % File 4
    { {'Actual_X', 'Actual_Y', 'Actual_Z'} },                     % File 5
    { {'Actual_X', 'Actual_Y', 'Actual_Z'} },                     % File 6
    { {'Actual_X', 'Actual_Y', 'Actual_Z'} },                     % File 7
    { {'Actual_X', 'Actual_Y', 'Actual_Z'} },                     % File 8
    { {'Actual_X', 'Actual_Y', 'Actual_Z'} },                     % File 9
    { {'Actual_X', 'Actual_Y', 'Actual_Z'} },                     % File 10
    { {'Actual_X', 'Actual_Y', 'Actual_Z'} }                      % File 11
};

% 3. Eksekusi fungsi
plot_multiple_trajectories_from_csv(files_to_plot, headers_to_plot);