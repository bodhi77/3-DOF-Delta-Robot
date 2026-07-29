%% plot_ch5_cost_tradeoff.m
% Figure 5-6: multi-objective cost trade-off (ch5, Eq. cost_function)
%   J(mu) = ebar/eta_e + lambda * max|dtau|/eta_tau
% Plots the tracking-cost term (rises with mu), the chattering-cost term
% (falls with mu), and the total cost J with its interior optimum mu*.
%
% NOTE: ebar(mu) and max|dtau|(mu) below are ILLUSTRATIVE monotonic models.
% The real curves come from your grid-search simulation -- replace the two
% model lines (a*mu and b./mu) with your measured data to get the true plot.
% Plots only; nothing is saved.

clear; clc; close all;

% ---- cost constants (from ch5) ----
eta_e   = 0.5;      % deg,  tracking normalization
eta_tau = 5.0;      % Nm,   torque normalization
lambda  = 0.05;     % chattering weight

% ---- illustrative dependence on mu (tune to your data) ----
a = 100;            % ebar(mu)     = a*mu   (deg)  -> tracking error grows with mu
b = 0.3;            % max|dtau|(mu)= b/mu    (Nm)   -> torque steps shrink with mu

mu = linspace(0.001, 0.006, 500);     % evolution-parameter range

ebar     = a * mu;
dtau_max = b ./ mu;

trackCost   = ebar / eta_e;
chatterCost = lambda * dtau_max / eta_tau;
J           = trackCost + chatterCost;

[Jmin, idx] = min(J);
muStar = mu(idx);

% ---- plot ----
figure('Color','w','Units','centimeters','Position',[2 2 15 10]);
hold on; box on; grid on;
plot(mu, trackCost,   'LineWidth', 2,   'Color', [0.20 0.45 0.80]);
plot(mu, chatterCost, 'LineWidth', 2,   'Color', [0.85 0.33 0.10]);
plot(mu, J,           'LineWidth', 2.5, 'Color', [0.10 0.10 0.10]);
plot([muStar muStar], [0 Jmin], '--', 'Color', [0.4 0.4 0.4], 'HandleVisibility','off');
plot(muStar, Jmin, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
text(muStar, Jmin, sprintf('  $\\mu^* = %.4f$', muStar), ...
     'Interpreter','latex', 'FontSize', 11, 'VerticalAlignment','top');

xlabel('Evolution parameter $\mu$', 'Interpreter','latex', 'FontSize', 13);
ylabel('Normalized cost', 'FontSize', 13);
legend({'Tracking cost $\bar{e}/\eta_e$', ...
        'Chattering cost $\lambda\,\max|\Delta\tau|/\eta_\tau$', ...
        'Total cost $J(\mu)$', 'Optimum $\mu^*$'}, ...
        'Interpreter','latex', 'Location','best', 'FontSize', 11);
set(gca, 'FontSize', 11, 'LineWidth', 1);
hold off;
