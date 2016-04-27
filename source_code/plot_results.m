% Requires method_name variable
columnwidth = 8.63;
textwidth = 17.78;

fig = figure(1);
set(gca, 'FontSize', 10)

hold on
plot(e_interval, S_cost)
grid on
xlabel('\epsilon')
ylabel('trace(S_0)')

fig.Units = 'centimeters';
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 columnwidth 0.7 * columnwidth];
fig.PaperSize = [columnwidth 0.7 * columnwidth];

print -dpng -r300 gcc_cost.png

fig = figure(2);
set(gca, 'FontSize', 10)

plot(0:N-1, Delta)
grid on
xlabel('Timestep - k')
ylabel('Disturbance norm - ||\Delta||_2')

fig.Units = 'centimeters';
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 textwidth / 3 textwidth / 3];
fig.PaperSize = [textwidth / 3 textwidth / 3];

print -dpng -r300 results_1.png

fig = figure(3);
set(gca, 'FontSize', 10)

hold on
plot(0:N-1, X(1:end-1,1))
plot(0:N-1, X(1:end-1,2), '--')
plot(0:N-1, X(1:end-1,3), '-.')
plot(0:N-1, ones(size(X(1:end-1,1))), 'k--')
plot(0:N-1, -ones(size(X(1:end-1,1))), 'k--')
ylim([-1.1, 1.1])

grid on
xlabel('Timestep - k')
ylabel('State - x_k')
legend('State 1', 'State 2', 'State 3', 'Boundary')

fig.Units = 'centimeters';
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 textwidth / 3 textwidth / 3];
fig.PaperSize = [textwidth / 3 textwidth / 3];

print -dpng -r300 results_2.png

fig = figure(4);
set(gca, 'FontSize', 10)

hold on
plot(0:N-1, U(:,1))
plot(0:N-1, U(:,2), '--')

grid on
xlabel('Timestep - k')
ylabel('Control Input - u_k')
legend('Input 1', 'Input 2')

fig.Units = 'centimeters';
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 textwidth / 3 textwidth / 3];
fig.PaperSize = [textwidth / 3 textwidth / 3];

print -dpng -r300 results_3.png