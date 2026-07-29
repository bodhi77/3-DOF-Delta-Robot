function T = calculate_mean_std()
    % Daftar file CSV
    % files = { ...
    %     '1000helixNeuroMamdaniT1-1.csv', '1000helixNeuroMamdaniT1-2.csv', '1000helixNeuroMamdaniT1-3.csv', ...
    %     '1000helixNeuroMamdaniT1-4.csv', '1000helixNeuroMamdaniT1-5.csv', '1000helixNeuroMamdaniT1-6.csv', ...
    %     '1000helixNeuroMamdaniT1-7.csv', '1000helixNeuroMamdaniT1-8.csv', '1000helixNeuroMamdaniT1-9.csv', ...
    %     '1000helixNeuroMamdaniT1-10.csv'};

    % files = { ...
    %     '1000helixNeuroIT2FS-1.csv', '1000helixNeuroIT2FS-2.csv', '1000helixNeuroIT2FS-3.csv', ...
    %     '1000helixNeuroIT2FS-4.csv', '1000helixNeuroIT2FS-5.csv', '1000helixNeuroIT2FS-6.csv', ...
    %     '1000helixNeuroIT2FS-7.csv', '1000helixNeuroIT2FS-8.csv', '1000helixNeuroIT2FS-9.csv', ...
    %     '1000helixNeuroIT2FS-10.csv'};

    % files = { ...
    % '0helixFuzzySynergeticT1-1.csv', '0helixFuzzySynergeticT1-2.csv', '0helixFuzzySynergeticT1-3.csv', ...
    % '0helixFuzzySynergeticT1-4.csv', '0helixFuzzySynergeticT1-5.csv', '0helixFuzzySynergeticT1-6.csv', ...
    % '0helixFuzzySynergeticT1-7.csv', '0helixFuzzySynergeticT1-8.csv', '0helixFuzzySynergeticT1-9.csv', ...
    % '0helixFuzzySynergeticT1-10.csv'};

    % files = { ...
    %     '1000helixFuzzySynergeticT2-1.csv', '1000helixFuzzySynergeticT2-2.csv', '1000helixFuzzySynergeticT2-3.csv', ...
    %     '1000helixFuzzySynergeticT2-4.csv', '1000helixFuzzySynergeticT2-5.csv', '1000helixFuzzySynergeticT2-6.csv', ...
    %     '1000helixFuzzySynergeticT2-7.csv', '1000helixFuzzySynergeticT2-8.csv', '1000helixFuzzySynergeticT2-9.csv', ...
    %     '1000helixFuzzySynergeticT2-10.csv'};

    % files = { ...
    %     '1000helixFuzzySynergeticT1-1.csv', '1000helixFuzzySynergeticT1-2.csv', '1000helixFuzzySynergeticT1-3.csv', ...
    %     '1000helixFuzzySynergeticT1-4.csv', '1000helixFuzzySynergeticT1-5.csv', '1000helixFuzzySynergeticT1-6.csv', ...
    %     '1000helixFuzzySynergeticT1-7.csv', '1000helixFuzzySynergeticT1-8.csv', '1000helixFuzzySynergeticT1-9.csv', ...
    %     '1000helixFuzzySynergeticT1-10.csv'};
    % 
     % files = { ...
     %    '1000helixSynergetic-1.csv', '1000helixSynergetic-2.csv', '1000helixSynergetic-3.csv', ...
     %    '1000helixSynergetic-4.csv', '1000helixSynergetic-5.csv', '1000helixSynergetic-6.csv', ...
     %    '1000helixSynergetic-7.csv', '1000helixSynergetic-8.csv', '1000helixSynergetic-9.csv', ...
     %    '1000helixSynergetic-10.csv'};

   % files = { ...
   %  '1000helixPID-1.csv', '1000helixPID-2.csv', '1000helixPID-3.csv', ...
   %  '1000helixPID-4.csv', '1000helixPID-5.csv', '1000helixPID-6.csv', ...
   %  '1000helixPID-7.csv', '1000helixPID-8.csv', '1000helixPID-9.csv', ...
   %  '1000helixPID-10.csv'};

  % files = { ...
  %   '1000helixFuzzyPIDT1-1.csv', '1000helixFuzzyPIDT1-2.csv', '1000helixFuzzyPIDT1-3.csv', ...
  %   '1000helixFuzzyPIDT1-4.csv', '1000helixFuzzyPIDT1-5.csv', '1000helixFuzzyPIDT1-6.csv', ...
  %   '1000helixFuzzyPIDT1-7.csv', '1000helixFuzzyPIDT1-8.csv', '1000helixFuzzyPIDT1-9.csv', ...
  %   '1000helixFuzzyPIDT1-10.csv'};

    % files = { ...
    %     '1000helixFuzzyPIDT2-1.csv', '1000helixFuzzyPIDT2-2.csv', '1000helixFuzzyPIDT2-3.csv', ...
    %     '1000helixFuzzyPIDT2-4.csv', '1000helixFuzzyPIDT2-5.csv', '1000helixFuzzyPIDT2-6.csv', ...
    %     '1000helixFuzzyPIDT2-7.csv', '1000helixFuzzyPIDT2-8.csv', '1000helixFuzzyPIDT2-9.csv', ...
    %     '1000helixFuzzyPIDT2-10.csv'};

    % files = { ...
    %     '1000helixSMC-1.csv', '1000helixSMC-2.csv', '1000helixSMC-3.csv', ...
    %     '1000helixSMC-4.csv', '1000helixSMC-5.csv', '1000helixSMC-6.csv', ...
    %     '1000helixSMC-7.csv', '1000helixSMC-8.csv', '1000helixSMC-9.csv', ...
    %     '1000helixSMC-10.csv'};

    % files = { ...
    %     '1000helixFuzzySMCT1-1.csv', '1000helixFuzzySMCT1-2.csv', '1000helixFuzzySMCT1-3.csv', ...
    %     '1000helixFuzzySMCT1-4.csv', '1000helixFuzzySMCT1-5.csv', '1000helixFuzzySMCT1-6.csv', ...
    %     '1000helixFuzzySMCT1-7.csv', '1000helixFuzzySMCT1-8.csv', '1000helixFuzzySMCT1-9.csv', ...
    %     '1000helixFuzzySMCT1-10.csv'};

    % files = { ...
    %     '1000helixFuzzySMCT2-1.csv', '1000helixFuzzySMCT2-2.csv', '1000helixFuzzySMCT2-3.csv', ...
    %     '1000helixFuzzySMCT2-4.csv', '1000helixFuzzySMCT2-5.csv', '1000helixFuzzySMCT2-6.csv', ...
    %     '1000helixFuzzySMCT2-7.csv', '1000helixFuzzySMCT2-8.csv', '1000helixFuzzySMCT2-9.csv', ...
    %     '1000helixFuzzySMCT2-10.csv'};



    

    % Header error & torsi (Kt = 1 -> torsi = arus)
    error_headers  = {'e1', 'e2', 'e3'};
    torque_headers = {'ActualCurr1', 'ActualCurr2', 'ActualCurr3'};
    num_files = length(files);

    % Hasil per-file (string utk format "mean ± std")
    mean_std_combined = strings(num_files, 1);
    rmse_values       = strings(num_files, 1);
    tau_rms_str       = strings(num_files, 1);
    tau_chatter_str   = strings(num_files, 1);

    % Akumulator global
    all_errors   = [];
    all_tau_rms_per_joint     = [];  % tiap baris = [rms1 rms2 rms3] per file
    all_tau_chatter_per_joint = [];  % tiap baris = [c1 c2 c3] per file

    for i = 1:num_files
        try
            data = readtable(files{i});

            % ===== ERROR (e1,e2,e3) =====
            error_data = [];
            for h = 1:numel(error_headers)
                col_name = error_headers{h};
                if ismember(col_name, data.Properties.VariableNames)
                    col_data = data.(col_name);
                    col_data = col_data(~isnan(col_data));
                    col_data = col_data * (180/pi);   % rad -> deg
                    error_data = [error_data; col_data];
                else
                    warning('Kolom %s tidak ditemukan di file %s.', col_name, files{i});
                end
            end

            % ===== TORSI per joint (Kt=1 -> tau = ActualCurr) =====
            tau_rms_joint     = nan(1, 3);
            tau_chatter_joint = nan(1, 3);
            for h = 1:numel(torque_headers)
                col_name = torque_headers{h};
                if ismember(col_name, data.Properties.VariableNames)
                    tau = data.(col_name);
                    tau = tau(~isnan(tau));
                    if isempty(tau), continue; end
                    % RMS torsi (effort)
                    tau_rms_joint(h) = sqrt(mean(tau.^2));
                    % Chattering: rata-rata |Delta-tau| antar langkah
                    if numel(tau) > 1
                        tau_chatter_joint(h) = mean(abs(diff(tau)));
                    end
                else
                    warning('Kolom %s tidak ditemukan di file %s.', col_name, files{i});
                end
            end

            if isempty(error_data)
                warning('Tidak ada data error valid di file %s.', files{i});
                mean_std_combined(i) = "ERROR";
                rmse_values(i)       = "ERROR";
                tau_rms_str(i)       = "ERROR";
                tau_chatter_str(i)   = "ERROR";
                continue;
            end

            % Statistik error per file
            mean_abs = mean(abs(error_data));
            std_dev  = std(error_data);
            rmse     = sqrt(mean(error_data.^2));

            % Statistik torsi per file (rata-rata 3 joint)
            tau_rms_avg     = mean(tau_rms_joint, 'omitnan');
            tau_chatter_avg = mean(tau_chatter_joint, 'omitnan');

            mean_std_combined(i) = sprintf('%.6f ± %.6f', mean_abs, std_dev);
            rmse_values(i)       = sprintf('%.6f', rmse);
            tau_rms_str(i)       = sprintf('%.4f', tau_rms_avg);
            tau_chatter_str(i)   = sprintf('%.4f', tau_chatter_avg);

            % Kumpulkan ke akumulator global
            all_errors = [all_errors; error_data];
            all_tau_rms_per_joint(end+1, :)     = tau_rms_joint;     %#ok<AGROW>
            all_tau_chatter_per_joint(end+1, :) = tau_chatter_joint; %#ok<AGROW>

        catch ME
            warning('Gagal membaca/memproses file %s: %s', files{i}, ME.message);
            mean_std_combined(i) = "ERROR";
            rmse_values(i)       = "ERROR";
            tau_rms_str(i)       = "ERROR";
            tau_chatter_str(i)   = "ERROR";
        end
    end

    % ===== Statistik global =====
    mean_all = mean(abs(all_errors));
    std_all  = std(all_errors);
    rmse_all = sqrt(mean(all_errors.^2));
    mean_std_global = sprintf('%.6f ± %.6f', mean_all, std_all);
    rmse_global     = sprintf('%.6f', rmse_all);

    tau_rms_global     = mean(all_tau_rms_per_joint(:),     'omitnan');
    tau_chatter_global = mean(all_tau_chatter_per_joint(:), 'omitnan');
    tau_rms_global_str     = sprintf('%.4f', tau_rms_global);
    tau_chatter_global_str = sprintf('%.4f', tau_chatter_global);

    % ===== Tabel akhir =====
    T = table(files', mean_std_combined, rmse_values, tau_rms_str, tau_chatter_str, ...
              'VariableNames', {'File', 'MeanAbs ± StdDev (deg)', 'RMSE (deg)', ...
                                'Tau_RMS (A)', 'Tau_Chatter (A)'});
    T = [T; {'Average', mean_std_global, rmse_global, tau_rms_global_str, tau_chatter_global_str}];

    writetable(T, 'tabel_performa.xls');

    assignin('base', 'T', T);
    assignin('base', 'mean_abs_global', mean_all);
    assignin('base', 'std_global', std_all);
    assignin('base', 'rmse_global', rmse_all);
    assignin('base', 'tau_rms_global', tau_rms_global);
    assignin('base', 'tau_chatter_global', tau_chatter_global);

    disp(T);
end

% Jalankan fungsi
T = calculate_mean_std();