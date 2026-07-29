function plot_multiple_csv_data(filenames, headers_list, dataTime)
    % =========================================================================
    % PENGATURAN SAKLAR INSET (ZOOM & ANOTASI)
    % =========================================================================
    inset_status = 'ON'; % Ubah menjadi 'OFF' untuk menonaktifkan inset dan panah
    % =========================================================================

    % Memastikan jumlah file dan header sesuai
    if length(filenames) ~= length(headers_list)
        error('Jumlah file dan daftar header harus sama.');
    end
    
    % =========================================================================
    % 1. PENGATURAN DUMMY INSET PLOT (ZOOM) 
    % =========================================================================
    % Batas area data yang ingin di-zoom: [X_min, X_max, Y_min, Y_max]
    zoom_limits = {
        [2.25, 2.75, 2, 12],   % DUMMY untuk Joint 1
        [2.25, 2.75, -2, 8],   % DUMMY untuk Joint 2
        [2.25, 2.75, 3, 13]    % DUMMY untuk Joint 3
    };
    % Posisi kotak zoom di layar: [X_kiri, Y_bawah, Lebar, Tinggi]
    inset_positions = {
        [0.3475, 0.60, 0.12, 0.15],    % DUMMY posisi kotak di Joint 1
        [0.82, 0.60, 0.12, 0.15],      % DUMMY posisi kotak di Joint 2
        [0.3475, 0.1025, 0.12, 0.15]   % DUMMY posisi kotak di Joint 3
    };
    % =========================================================================
    
    num_files = length(filenames);
    colors = distinguishable_colors(num_files); 
    
    % =========================================================================
    % 2. KUMPULAN GAYA KOMBINASI UNIK (LINESTYLE + MARKER)
    % =========================================================================
    styles_pool =  {'-',  ':',  '-.', '-',  ':',  '-.', '-',  ':',  '-.',  '-',  ':'};
    % markers_pool = {'none','none','none','*',  '+',  '>',  'x',  's',  '^',  'D',  '*'};
    markers_pool = {'none','none','none','none','none','none', 'none','none','none', 'none','none'};
    % =========================================================================
    
    figure('Name', 'Step Response', 'Units', 'normalized', 'Position', [0.1, 0.1, 0.8, 0.8]);
    t = tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    
    h_lines = gobjects(0);
    h_labels = {};
    method_plotted = false(1, num_files);
    ref_plotted = false;
    
    % Array untuk menyimpan handle dari axes utama agar bisa dipanggil lagi nanti
    ax_mains = gobjects(3, 1);
    
    % --- LOOP 1: MENGGAMBAR SEMUA GRAFIK & KOTAK INSET TERLEBIH DAHULU ---
    for joint_idx = 1:3
        nexttile(t);
        ax_main = gca; 
        ax_mains(joint_idx) = ax_main; % Simpan handle axes
        
        hold(ax_main, 'on');
        grid(ax_main, 'on');
        
        title(sprintf('Joint %d', joint_idx), 'FontSize', 16);
        xlabel('Time (s)', 'FontSize', 14);
        ylabel('Angle (Degree)', 'FontSize', 14);
        ax_main.FontSize = 14;
        
        % --- PLOT DATA UTAMA ---
        for f = 1:num_files
            data = readtable(filenames{f});
            [~, method_name, ~] = fileparts(filenames{f});
            method_name = strrep(method_name, '0helix', ''); 
            method_name = strrep(method_name, '-1', '');
            
            % Mengambil kombinasi gaya unik untuk file saat ini
            idx_gaya = mod(f-1, length(styles_pool)) + 1;
            current_style = styles_pool{idx_gaya};
            current_marker = markers_pool{idx_gaya};
            
            N = height(data);
            time = linspace(0, dataTime, N)'; 
            
            % Memunculkan marker sebanyak 45 titik di sepanjang grafik
            marker_spacing = round(linspace(1, N, 45));
            
            for i = 1:length(headers_list{f})
                header_name = char(headers_list{f}{i}); 
                if endsWith(header_name, num2str(joint_idx))
                    is_reference = contains(header_name, 'Set', 'IgnoreCase', true) || ...
                                   contains(header_name, 'ref', 'IgnoreCase', true);
                    
                    if is_reference
                        % Referensi: Hitam tebal tanpa marker
                        p_ref = plot(ax_main, time, data.(header_name), 'k-', 'LineWidth', 2.5);
                        if ~ref_plotted
                            h_lines = [p_ref, h_lines]; 
                            h_labels = [{'Reference'}, h_labels];
                            ref_plotted = true;
                        end
                    else
                        % Metode: Kombinasi warna, tipe garis, dan marker terkontrol
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
        xlim(ax_main, [0, dataTime]);
        ylim(ax_main, [-20, 60]);
        hold(ax_main, 'off');
        
        % =====================================================================
        % BLOK KONDISI INSET
        % =====================================================================
        if strcmpi(inset_status, 'ON')
            % --- MEMBUAT INSET PLOT (ZOOM BOX) ---
            z_lim = zoom_limits{joint_idx};
            
            % Gambar kotak penanda di plot utama
            rectangle(ax_main, 'Position', [z_lim(1), z_lim(3), z_lim(2)-z_lim(1), z_lim(4)-z_lim(3)], ...
                      'EdgeColor', 'k', 'LineWidth', 1, 'LineStyle', '-');
                  
            % Buat kanvas baru (axes) untuk inset
            ax_inset = axes('Position', inset_positions{joint_idx});
            hold(ax_inset, 'on');
            box(ax_inset, 'on');
            grid(ax_inset, 'on');
            
            % Copy semua garis (Otomatis membawa properti LineStyle dan MarkerIndices-nya)
            garis_plot = findobj(ax_main, 'Type', 'Line');
            copyobj(garis_plot, ax_inset);
            
            % Set limit sumbu inset
            xlim(ax_inset, [z_lim(1), z_lim(2)]);
            ylim(ax_inset, [z_lim(3), z_lim(4)]);
            ax_inset.FontSize = 10;
        end
        % =====================================================================
    end
    
    % PERINTAH PENTING: Memaksa MATLAB merender seluruh layout secara final. 
    drawnow; 
    
    % =========================================================================
    % BLOK KONDISI ANOTASI PANAH
    % =========================================================================
    if strcmpi(inset_status, 'ON')
        % --- LOOP 2: MENGGAMBAR PANAH PENGHUBUNG ---
        for joint_idx = 1:3
            ax_main = ax_mains(joint_idx);
            z_lim = zoom_limits{joint_idx};
            ins = inset_positions{joint_idx};
            
            xl = xlim(ax_main);
            yl = ylim(ax_main);
            
            ax_main.Units = 'normalized';
            ax_pos = ax_main.Position;
            
            data2figX = @(xd) ax_pos(1) + (xd - xl(1)) / (xl(2) - xl(1)) * ax_pos(3);
            data2figY = @(yd) ax_pos(2) + (yd - yl(1)) / (yl(2) - yl(1)) * ax_pos(4);
            
            % TITIK AWAL: Right-Center dari kotak hitam
            zRC_X = data2figX(z_lim(2)); 
            zRC_Y = data2figY((z_lim(3) + z_lim(4)) / 2); 
            
            % TITIK AKHIR: Left-Center dari jendela inset
            iLC_X = ins(1); 
            iLC_Y = ins(2) + (ins(4) / 2); 
            
            annotation('arrow', [zRC_X, iLC_X], [zRC_Y, iLC_Y], 'Color', 'k', 'LineWidth', 1, 'HeadStyle', 'vback2');
        end
    end
    % =========================================================================
    
    % Eksekusi legend global
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

% --- CONTOH PEMANGGILAN ---
% plot_multiple_csv_data({['160_impulse_s1_NeuroMamdaniT1-SIM.csv'], ...}, {...}, 7);
% plot_multiple_csv_data({['0helixNeuroMamdaniT1-1.csv'], ['0helixNeuroIT2FS-1.csv'], ['0helixSynergetic-1.csv'],...
%     ['0helixFuzzySynergeticT1-1.csv'], ['0helixFuzzySynergeticT2-1.csv'], ['0helixPID-1.csv'], ...
%     ['0helixFuzzyPIDT1-1.csv'], ['0helixFuzzyPIDT2-1.csv' ], ['0helixSMC-1.csv'], ...
%     ['0helixFuzzySMCT1-1.csv'],['0helixFuzzySMCT2-1.csv']}, { 
%     {'ActualAngle1','ActualAngle2','ActualAngle3'},
%     {'ActualAngle1','ActualAngle2','ActualAngle3'},
%     {'ActualAngle1','ActualAngle2','ActualAngle3'},
%     {'ActualAngle1','ActualAngle2','ActualAngle3'},
%     {'ActualAngle1','ActualAngle2','ActualAngle3'},
%     {'ActualAngle1','ActualAngle2','ActualAngle3'},
%     {'ActualAngle1','ActualAngle2','ActualAngle3'},
%     {'ActualAngle1','ActualAngle2','ActualAngle3'},
%     {'ActualAngle1','ActualAngle2','ActualAngle3'},
%     {'ActualAngle1','ActualAngle2','ActualAngle3'},
%     {'ActualAngle1','ActualAngle2','ActualAngle3'}
% }, 7);

% plot_multiple_csv_data({['0helixNeuroMamdaniT1-1.csv'], ['0helixNeuroIT2FS-1.csv'], ['0helixSynergetic-1.csv'],...
%     ['0helixFuzzySynergeticT1-1.csv'], ['0helixFuzzySynergeticT2-1.csv']}, { 
%     {'mu1','mu2','mu3'},
%     {'mu1','mu2','mu3'},
%     {'mu1','mu2','mu3'},
%     {'mu1','mu2','mu3'},   
%     {'mu1','mu2','mu3'}
% }, 7);

% plot_multiple_csv_data({['160_impulse_s1_NeuroMamdaniT1-SIM.csv'], ['160_impulse_s1_NeuroIT2FS-SIM.csv'] ,['160_impulse_s1_FuzzyT1-SIM.csv'], ...
%     ['160_impulse_s1_FuzzyT2-SIM'], ['160_impulse_s1_StaticMu-SIM.csv']}, { 
%     {'SetAngle1', 'SetAngle2', 'SetAngle3','ActualAngle1','ActualAngle2','ActualAngle3'},
%     {'ActualAngle1','ActualAngle2','ActualAngle3'},
%     {'ActualAngle1','ActualAngle2','ActualAngle3'},
%     {'ActualAngle1','ActualAngle2','ActualAngle3'},
%     {'ActualAngle1','ActualAngle2','ActualAngle3'}
% }, 5);

plot_multiple_csv_data({['160_impulse_s2_NeuroMamdaniT1-SIM.csv'], ['160_impulse_s2_NeuroIT2FS-SIM.csv'] ,['160_impulse_s2_FuzzyT1-SIM.csv'], ...
    ['160_impulse_s2_FuzzyT2-SIM'], ['160_impulse_s2_StaticMu-SIM.csv']}, { 
    {'SetAngle1', 'SetAngle2', 'SetAngle3','ActualAngle1','ActualAngle2','ActualAngle3'},
    {'ActualAngle1','ActualAngle2','ActualAngle3'},
    {'ActualAngle1','ActualAngle2','ActualAngle3'},
    {'ActualAngle1','ActualAngle2','ActualAngle3'},
    {'ActualAngle1','ActualAngle2','ActualAngle3'}
}, 5);