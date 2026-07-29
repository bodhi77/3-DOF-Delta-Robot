function plot_multiple_trajectories_from_csv(filenames, headers)
    % Memastikan jumlah file dan header sesuai
    if length(filenames) ~= length(headers)
        error('Jumlah file dan header harus sama.');
    end
    
    colors = lines(sum(cellfun(@(x) length(x), headers))); % Warna berbeda untuk setiap trajectory
    color_idx = 1; % Indeks warna untuk setiap trajectory
    
    figure;
    hold on;
    
    for f = 1:length(filenames)
        % Membaca file CSV
        data = readtable(filenames{f});
        
        % Memeriksa apakah headers{f} adalah sel array atau langsung list header
        if iscell(headers{f}{1})
            num_trajectories = length(headers{f});
        else
            num_trajectories = 1;
            headers{f} = {headers{f}}; % Ubah menjadi bentuk nested cell agar seragam
        end
        
        for t = 1:num_trajectories
            % Memastikan semua header dalam trajectory ada dalam file CSV
            for i = 1:length(headers{f}{t})
                if ~ismember(headers{f}{t}{i}, data.Properties.VariableNames)
                    error('Header "%s" tidak ditemukan dalam file CSV.', headers{f}{t}{i});
                end
            end
            
            % Mengambil nilai X, Y, Z dari trajectory
            X = data.(headers{f}{t}{1});
            Y = data.(headers{f}{t}{2});
            Z = data.(headers{f}{t}{3});
            
            % Plot tiap trajectory dengan warna unik
            plot3(X, Y, Z, 'Color', colors(color_idx, :), 'LineWidth', 2, 'DisplayName', sprintf(filenames{f}));
            color_idx = color_idx + 1;
        end
    end
    
    xlabel('X (mm)');
    ylabel('Y (mm)');
    zlabel('Z (mm)');
    title('Trajectory Plot');
    grid on;
    axis equal;
    view(3);
    legend;
    
    hold off;
end

% Contoh pemanggilan fungsi untuk multiple files dan multiple trajectories
% plot_multiple_trajectories_from_csv({['PID9.csv'], ['Fuzzy9.csv'], ['SMC9.csv']}, { 
%     { {'X0', 'Y0', 'Z0'}, {'Actual_X', 'Actual_Y', 'Actual_Z'} },
%     { {'Actual_X', 'Actual_Y', 'Actual_Z'} },
%     { {'Actual_X', 'Actual_Y', 'Actual_Z'} }
% });

% plot_multiple_trajectories_from_csv({['0paraSynergetic-.csv']}, { 
%     { {'X0', 'Y0', 'Z0'}, {'Actual_X', 'Actual_Y', 'Actual_Z'} },
% });

plot_multiple_trajectories_from_csv({['160_0helixNeuroMamdaniT1-SIM.csv']}, { 
    { {'X0', 'Y0', 'Z0'}, {'Actual_X', 'Actual_Y', 'Actual_Z'} },
});