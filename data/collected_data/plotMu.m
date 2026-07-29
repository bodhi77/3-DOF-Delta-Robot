function plot_multiple_csv_data(filenames, headers_list, dataTime)
    % Memastikan jumlah file dan header sesuai
    if length(filenames) ~= length(headers_list)
        error('Jumlah file dan daftar header harus sama.');
    end
    
    num_files = length(filenames);
    colors = distinguishable_colors(num_files); 
    
    % =========================================================================
    % KUMPULAN GAYA KOMBINASI UNIK (LINESTYLE + MARKER TERKONTROL)
    % =========================================================================
    styles_pool =  {'-',  '-.',  ':', '-',  '-.',  ':',  '-',  '-.', ':',  '-',  '-.'};
    % markers_pool = {'none','none','none','o',  '>',  'y',  'x',  's',  '^',  'D',  '*'};
    markers_pool = {'none','none','none','none','none','none', 'none','none','none','none','none'};
    % =========================================================================
    
    figure('Name', 'Step Response', 'Units', 'normalized', 'Position', [0.1, 0.1, 0.8, 0.8]);
    t = tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    
    h_lines = gobjects(0);
    h_labels = {};
    method_plotted = false(1, num_files);
    ref_plotted = false;
    
    % --- LOOP: MENGGAMBAR GRAFIK UTAMA ---
    for joint_idx = 1:3
        nexttile(t);
        ax_main = gca; 
        
        hold(ax_main, 'on');
        grid(ax_main, 'on');
        
        title(sprintf('Joint %d', joint_idx), 'FontSize', 16);
        xlabel('Time (s)', 'FontSize', 14);
        ylabel('Amplitude', 'FontSize', 14); 
        ax_main.FontSize = 14;
        
        for f = 1:num_files
            data = readtable(filenames{f});
            [~, method_name, ~] = fileparts(filenames{f});
            method_name = strrep(method_name, '1000helix', ''); 
            method_name = strrep(method_name, '-1', '');
            
            % Mengambil kombinasi gaya unik
            idx_gaya = mod(f-1, length(styles_pool)) + 1;
            current_style = styles_pool{idx_gaya};
            current_marker = markers_pool{idx_gaya};
            
            N = height(data);
            time = linspace(0, dataTime, N)'; 
            marker_spacing = round(linspace(1, N, 45));
            
            for i = 1:length(headers_list{f})
                header_name = char(headers_list{f}{i}); 
                if endsWith(header_name, num2str(joint_idx))
                    is_reference = contains(header_name, 'Set', 'IgnoreCase', true) || ...
                                   contains(header_name, 'ref', 'IgnoreCase', true);
                    
                    if is_reference
                        p_ref = plot(ax_main, time, data.(header_name), 'k-', 'LineWidth', 2.5);
                        if ~ref_plotted
                            h_lines = [p_ref, h_lines]; 
                            h_labels = [{'Reference'}, h_labels];
                            ref_plotted = true;
                        end
                    else
                        p_method = plot(ax_main, time, data.(header_name), 'Color', colors(f, :), ...
                                        'LineWidth', 1.5, 'LineStyle', current_style, ...
                                        'Marker', current_marker, 'MarkerSize', 4, ...
                                        'MarkerIndices', marker_spacing);
                        if ~method_plotted(f)
                            h_lines(end+1) = p_method;
                            h_labels{end+1} = method_name;
                            method_plotted(f) = true;
                        end
                    end
                end
            end
        end
        
        % Hanya membatasi sumbu X. Sumbu Y dibiarkan bebas (Auto-Scale)
        xlim(ax_main, [0, dataTime]);
        ylim(ax_main, [0, 0.006])
        hold(ax_main, 'off');
    end
    
    % --- EKSEKUSI LEGEND ---
    if ~isempty(h_lines)
        lgd = legend(h_lines, h_labels);
        lgd.Layout.Tile = 4;
        lgd.NumColumns = 2; 
        lgd.FontSize = 14;
    end
end

function colors = distinguishable_colors(n)
    colors = hsv(n) * 0.8;
end

% =========================================================================
% BLOK PEMANGGILAN UNTUK DATA MU
% =========================================================================
% plot_multiple_csv_data({['160_1000helixNeuroMamdaniT1-SIM.csv'], ['160_1000helixNeuroIT2FS-SIM.csv'], ['160_1000helixSynergetic-SIM.csv'],...
%     ['160_1000helixFuzzySynergeticT1-SIM.csv'], ['160_1000helixFuzzySynergeticT2-SIM.csv']}, { 
%     {'mu1','mu2','mu3'},
%     {'mu1','mu2','mu3'},
%     {'mu1','mu2','mu3'},
%     {'mu1','mu2','mu3'},   
%     {'mu1','mu2','mu3'}
% }, 7);

