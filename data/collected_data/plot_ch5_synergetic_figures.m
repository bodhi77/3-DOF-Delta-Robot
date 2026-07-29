%% plot_ch5_synergetic_figures.m
% Plots the two synergetic-control illustration figures for Chapter 5.
%   Figure 5-1 : LINEAR manifold  sigma = c*e + edot = 0  in the (e, edot)
%                phase plane, for several values of the constant c.
%                Your manifold is linear (single c) -> STRAIGHT lines.
%                Auzan's curves are bent because he adds a nonlinear
%                c2*|e|^(p/q) term (terminal manifold), not used here.
%   Figure 5-2 : macrovariable convergence  sigma(t)/sigma(0) = exp(-t/mu),
%                from the evolution constraint  mu*sigmadot + sigma = 0.
% Plots only; nothing is saved (edit freely, then export yourself).

clear; clc; close all;

%% ---------- Figure 5-1: linear manifold sigma = c*e + edot = 0 ----------
% Your design manifold constant is c = 80; vary around it to show the effect.
cValues = [20 40 60 80 100];
e = linspace(-0.1, 0.1, 400);         % joint position error axis (rad)

figure('Color','w','Units','centimeters','Position',[2 2 14 10]);
hold on; box on; grid on;
cmap = lines(numel(cValues));
h = gobjects(1, numel(cValues));
for k = 1:numel(cValues)
    edot = -cValues(k) * e;           % sigma = 0  ->  edot = -c*e
    h(k) = plot(e, edot, 'LineWidth', 2, 'Color', cmap(k,:));
end
plot([-0.1 0.1], [0 0], 'k-', 'LineWidth', 0.5, 'HandleVisibility','off');
plot([0 0], [-10 10], 'k-', 'LineWidth', 0.5, 'HandleVisibility','off');
xlabel('Joint position error $e$ (rad)', 'Interpreter','latex', 'FontSize', 13);
ylabel('Joint velocity error $\dot{e}$ (rad/s)', 'Interpreter','latex', 'FontSize', 13);
legend(h, compose('$c = %g$', cValues), 'Interpreter','latex', ...
       'Location','northeast', 'FontSize', 11);
axis([-0.1 0.1 -10 10]);
set(gca, 'FontSize', 11, 'LineWidth', 1);
hold off;

%% ---------- Figure 5-2: sigma(t)/sigma0 = exp(-t/mu) ----------
muValues = [0.001 0.003 0.006];       % evolution parameter (s), your range
t = linspace(0, 0.03, 500);           % time (s)

figure('Color','w','Units','centimeters','Position',[2 2 14 10]);
hold on; box on; grid on;
cmap2 = lines(numel(muValues));
h2 = gobjects(1, numel(muValues));
for k = 1:numel(muValues)
    sigma = exp(-t / muValues(k));    % normalized macrovariable, sigma(0)=1
    h2(k) = plot(t*1e3, sigma, 'LineWidth', 2, 'Color', cmap2(k,:));
end
xlabel('Time (ms)', 'FontSize', 13);
ylabel('$\sigma(t)/\sigma(0)$', 'Interpreter','latex', 'FontSize', 13);
legend(h2, compose('$\\mu = %.3f$ s', muValues), 'Interpreter','latex', ...
       'Location','northeast', 'FontSize', 11);
axis([0 30 0 1]);
set(gca, 'FontSize', 11, 'LineWidth', 1);
hold off;
