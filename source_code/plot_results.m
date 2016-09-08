%%
columnwidth = 8.63;
textwidth = 17.78;

%%
fig = figure(1);
set(gca, 'FontSize', 10)

hold on
plot(e_interval, S_cost)
grid on
xlabel('\epsilon')
ylabel('trace(S_0)')

fig.Units = 'centimeters';
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 columnwidth 0.9 * columnwidth];
fig.PaperSize = [columnwidth 0.9 * columnwidth];

print -dpng -r300 gcc_cost.png

%%
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

%%
fig = figure(3);
set(gca, 'FontSize', 10)

hold on
h1 = plot(0:N-1, X(1:end-1,1), 'b-');
plot(0:N-1, X(1:end-1,2), 'b-')
plot(0:N-1, X(1:end-1,3), 'b-')
h2 = plot(0:N-1, X1(1:end-1,1), 'r--');
plot(0:N-1, X1(1:end-1,2), 'r--')
plot(0:N-1, X1(1:end-1,3), 'r--')
h3 = plot(0:N-1, ones(size(X(1:end-1,1))), 'k--');
plot(0:N-1, -ones(size(X(1:end-1,1))), 'k--')
ylim([-1.1, 1.1])

grid on
xlabel('Timestep - k')
ylabel('State - x_k')
legend([h1, h2, h3], 'GCMPC States', 'Enumeration-based RMPC states', 'Boundary')

fig.Units = 'centimeters';
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 textwidth / 3 textwidth / 3];
fig.PaperSize = [textwidth / 3 textwidth / 3];

print -dpng -r300 results_2.png

%%
fig = figure(4);
set(gca, 'FontSize', 10)

hold on
h1 = plot(0:N-1, U(:,1), 'b-');
plot(0:N-1, U(:,2), 'b-')
h2 = plot(0:N-1, U(:,1), 'r--');
plot(0:N-1, U(:,2), 'r--')

grid on
xlabel('Timestep - k')
ylabel('Control Input - u_k')
legend([h1, h2], 'Input 1', 'Input 2', 'Location', 'southeast')

fig.Units = 'centimeters';
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 textwidth / 3 textwidth / 3];
fig.PaperSize = [textwidth / 3 textwidth / 3];

print -dpng -r300 results_3.png

%%
fig = figure(5);
set(gca, 'FontSize', 10)

boxplot(1000 * [gcmpc_time, rmpc_time], 'label', {'GCMPC', 'Enumeration-based RMPC'}, 'whisker', 100)

grid on
ylim([1, 1e4])
set(gca, 'yscale', 'log')
ylabel('Mosek solve time [ms]')

fig.Units = 'centimeters';
fig.PaperUnits = 'centimeters';
fig.PaperPosition = [0 0 columnwidth 0.9 * columnwidth];
fig.PaperSize = [columnwidth 0.9 * columnwidth];

print -dpng -r300 time_comparison.png