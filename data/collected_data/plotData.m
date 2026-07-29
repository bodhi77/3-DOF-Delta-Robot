function plot_multiple_csv_data(filenames, headers_list, dataTime)
    % Memastikan jumlah file dan header sesuai
    if length(filenames) ~= length(headers_list)
        error('Jumlah file dan daftar header harus sama.');
    end
    
    total_plots = sum(cellfun(@length, headers_list)); % Total jumlah plot
    colors = distinguishable_colors(total_plots); % Warna benar-benar berbeda untuk setiap plot
    color_idx = 1; % Indeks warna untuk setiap dataset
    
    figure;
    hold on;
    xlabel('Time (s)', 'FontSize',16);
    ylabel('Angle (Degree)', 'FontSize',16);
    title('Step Response', FontSize=16);

    ax = gca;
    ax.FontSize = 16; 
    grid on;
    
    for f = 1:length(filenames)
        % Membaca file CSV
        data = readtable(filenames{f});
        
        % Menampilkan nama header yang tersedia
        disp(['Headers dalam file ', filenames{f}, ':']);
        disp(data.Properties.VariableNames);
        
        % Memastikan semua header yang diminta ada dalam file
        for i = 1:length(headers_list{f})
            if ~ismember(headers_list{f}{i}, data.Properties.VariableNames)
                error('Header "%s" tidak ditemukan dalam file %s.', headers_list{f}{i}, filenames{f});
            end
        end
        
        % Menentukan jumlah sampel berdasarkan panjang file
        N = height(data); % Jumlah total data dalam CSV
        
        % Membuat sumbu waktu dari 0 hingga dataTime dengan jumlah titik N
        time = linspace(0, dataTime, N)'; 
        
        % Plot masing-masing data terhadap waktu dengan warna unik
        for i = 1:length(headers_list{f})
            plot(time, data.(headers_list{f}{i}), 'Color', colors(color_idx, :), 'LineWidth', 1.5, 'DisplayName', sprintf('%s - %s', filenames{f}, headers_list{f}{i}));
            color_idx = color_idx + 1;
        end
    end
    
    legend;
    lgd=legend;
    lgd.FontSize = 16;
    hold off;
    xlim([0, dataTime]);
    % ylim([-3, 3]);
end

function colors = distinguishable_colors(n)
    % Membuat warna yang benar-benar berbeda satu sama lain
    % colors = hsv(n); % Menggunakan spektrum warna HSV untuk distribusi optimal
    % colors = parula(n);
    colors = hsv(n)*0.8;
    % rng(42); % Seed untuk memastikan warna tetap berbeda setiap kali dijalankan
    % colors = rand(n, 3) * 0.9; % Warna acak dalam RGB, tetapi tidak terlalu terang

end

% Contoh penggunaan dengan beberapa file dan header yang berbeda:
% plot_multiple_csv_data({['0helixNeuroMamdaniT1-1.csv'], ['0helixNeuroIT2FS-1.csv'], ['0helixSynergetic-1.csv'],...
%     ['0helixFuzzySynergeticT1-1.csv'], ['0helixFuzzySynergeticT2-1.csv'], ['0helixPID-1.csv'], ...
%     ['0helixFuzzyPIDT1-1.csv'], ['0helixFuzzyPIDT2-1.csv' ], ['0helixSMC-1.csv'], ...
%     ['0helixSMC-1.csv'],['0helixFuzzySMCT1-1.csv'],['0helixFuzzySMCT2-1.csv']}, { 
%     {'SetAngle1', 'SetAngle2', 'SetAngle3', 'ActualAngle1','ActualAngle2','ActualAngle3'},
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

% plot_multiple_csv_data({['0paraSynergetic-.csv']}, { 
%        {'SetAngle1', 'SetAngle2', 'SetAngle3', 'ActualAngle1', 'ActualAngle2', 'ActualAngle3' }
%    }, 17);

% plot_multiple_csv_data({['0paraIT2FS-.csv']}, { 
%        {'SetAngle1', 'SetAngle2', 'SetAngle3', 'ActualAngle1', 'ActualAngle2', 'ActualAngle3' }
%    }, 17);

% plot_multiple_csv_data({['savedData.csv']}, { 
%        {'seti3', 'i3' }
%    }, 17);

% plot_multiple_csv_data({['0paraIT2FS-1.csv']}, { 
%      {'SetAngle1', 'SetAngle2', 'SetAngle3', 'ActualAngle1', 'ActualAngle2', 'ActualAngle3'}
%  }, 17);

% plot_multiple_csv_data({['0paraFuzzySMCT2-.csv']}, { 
%      {'SetAngle1', 'SetAngle2', 'SetAngle3', 'ActualAngle1', 'ActualAngle2', 'ActualAngle3'}
%  }, 17);

% plot_multiple_csv_data({['0paraFuzzySynergeticT2-1.csv']}, { 
%      {'refCurr1'}
%  }, 7);

% plot_multiple_csv_data({['0helixNeuroIT2FS-2.csv']}, { 
%      {'SetAngle1', 'SetAngle2', 'SetAngle3', 'ActualAngle1', 'ActualAngle2', 'ActualAngle3'}
%  }, 7);

% plot_multiple_csv_data({['0helixSynergetic-2.csv']}, { 
%      {'refCurr1', 'ActualCurr1'}
%  }, 7);


% plot_multiple_csv_data({['0paraSynergetic-.csv']}, { 
%      {'refCurr1', 'ActualCurr1'}
%  }, 7);

% plot_multiple_csv_data({['1000paraSMC-.csv']}, { 
%      {'refCurr1', 'ActualCurr1'}
%  }, 7);

% plot_multiple_csv_data({['1000helixNeuroMamdaniT1-1.csv']}, { 
%      {'refCurr1', 'ActualCurr1'}
%  }, 7);

% plot_multiple_csv_data({['SMC1.csv']}, { 
%       {'seti2',  'i2' }
%   }, 17);
% plot_multiple_csv_data({['savedData.csv']}, { 
%        {'Kp1', 'Kp2', 'Kp3' }
%    }, 17);


plot_multiple_csv_data({['0helixNeuroMamdaniT1-1.csv'], ['0helixNeuroIT2FS-1.csv'], ['0helixSynergetic-1.csv'],...
    ['0helixFuzzySynergeticT1-1.csv'], ['0helixFuzzySynergeticT2-1.csv']}, { 
    {'mu1','mu2','mu3'},
    {'mu1','mu2','mu3'},
    {'mu1','mu2','mu3'},
    {'mu1','mu2','mu3'},   
    {'mu1','mu2','mu3'}
}, 7);