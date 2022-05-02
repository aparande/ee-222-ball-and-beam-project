function plot_neglected_nonlinearity(ts, psis)

fig = figure();
% Set up font size.
set(fig, 'DefaultAxesFontSize', 16);
% Set up font name
set(fig, 'DefaultTextFontName', 'Times New Roman');
% Set up interpreter
set(fig, 'DefaultTextInterpreter', 'latex');

plot(ts, psis, 'LineWidth', 1.5);
ylabel('$\psi$');
xlabel('$t$ [sec]');
grid on;    
title('Neglected Nonlinearity Term $\psi$');

