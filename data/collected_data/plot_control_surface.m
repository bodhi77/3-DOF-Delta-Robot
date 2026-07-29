% plot_control_surface.m
% ---------------------------------------------------------------------
% STEP 2 (MATLAB). Draws the IT2-MNFS control surface:  mu = f(e, e_dot).
%
% Floor of the plot = the two inputs (error e, error rate de/dt).
% Height           = mu the trained model decided for that situation.
%
% Needs control_surface.csv, produced first by export_control_surface.py.
% ---------------------------------------------------------------------

T = readmatrix('control_surface.csv');   % header row is skipped automatically
e    = T(:,1);      % error (deg)
edot = T(:,2);      % error rate (deg/s)
mu   = T(:,3);      % model output

% Turn the scattered (e, edot, mu) points into a smooth grid for surf()
eu = linspace(min(e),    max(e),    120);
du = linspace(min(edot), max(edot), 120);
[E, D] = meshgrid(eu, du);
MU = griddata(e, edot, mu, E, D);

figure('Color','w');
surf(E, D, MU, 'EdgeColor','none');
shading interp;

% Menggunakan LaTeX interpreter untuk sumbu X, Y, Z, dan Title
xlabel('Error $e$ (deg)', 'Interpreter', 'latex', 'FontSize', 12);
ylabel('Error rate $\frac{de}{dt}$ (deg/s)', 'Interpreter', 'latex', 'FontSize', 12);
zlabel('$\mu$', 'Interpreter', 'latex', 'FontSize', 14);
title('\textbf{IT2-MNFS Learned Control Surface}', 'Interpreter', 'latex', 'FontSize', 14);

colorbar;
colormap(parula);
view(135, 30);
grid on;
axis tight;

% Optional: a flat top-down heatmap view instead of the 3-D hill --
% uncomment the lines below.
% figure('Color','w'); contourf(E, D, MU, 20, 'LineColor','none');
% xlabel('Error $e$ (deg)', 'Interpreter', 'latex', 'FontSize', 12); 
% ylabel('Error rate $\frac{de}{dt}$ (deg/s)', 'Interpreter', 'latex', 'FontSize', 12); 
% colorbar; 
% title('\textbf{$\mu$ Control Surface (Top View)}', 'Interpreter', 'latex', 'FontSize', 14);