% This is main file for simulation, post processing and plotting
clc; % Clears all the text from the Command Window.
close all;
clear; % Remove items from workspace, freeing up system memory

% Controller parameters
omega = [3 3 3];
k1 = [700, 0, 0; 0, 700, 0; 0, 0, 700];

k2 = [100, 0, 0; 0,100, 0; 0, 0, 100];
% k1 = diag(omega.^2);
% k2 = diag(2*omega);
% controller = system;
controller.k1 = k1;
controller.k2 = k2;
controller.b = 1.8;
controller.c = 3.5;
controller.d = 4.5;
controller.lambda = k2;
% Trajectory params
A = [3 1.5 2.0];
A0 = [2 6 3];
nu = [0.5 0.8 1.2];
trajectory.A = A;
trajectory.A0 = A0;
trajectory.nu = nu;

% Time settings
t0 = 0;
tf = 10;
dt = 0.1;
% Integration (simulation)
% Create a time vector for numerical integration
tspan = [t0:dt:tf]; 
% Set a vector of initial conditions x0 = [theta_0 dthetadt_0]
x0 = [2.0, 3.0, 4.0, 0.0, 0.0, 0.0]; 
% Run a solver to integrate our differential equations
[t,x] = ode45(@(t,x)dynamics(t, x, trajectory,  controller), tspan, x0);
% t - vector of time 
% x - solution of ODE (value of evalueted integral in time span t)

% Calculate control and desired trajectory for solution
for i = 1:length(x)
    [u_i, x_des_i] = control(t(i), x(i,:), trajectory, controller);
    u(i,:) = u_i;
    x_des(i,:) = x_des_i;
    error(i,:) = x_des_i - x(i,1:3);
end

% Ploting
% Set latex interpreter for plots text and legends
figure(1)
set(0, 'DefaultTextInterpreter', 'latex') 
set(0, 'DefaultLegendInterpreter', 'latex')
% Subplot time evalution of system state
clf;
subplot(2,1,1);
hold on;
% for i = 1:3
%     plot(t, x(:,i), 'r', 'linewidth', 1);
%     plot(t, error(:,i), 'g', 'linewidth', 1);
%     plot(t, x_des(:,i), 'b--', 'linewidth', 1);
% end
plot(t, x(:,1), 'r', 'linewidth', 1);
plot(t, error(:,1), 'g', 'linewidth', 1);
plot(t, x_des(:,1), 'b', 'linewidth', 1);

plot(t, x(:,2), 'r-.', 'linewidth', 0.05);
plot(t, error(:,2), 'g-.', 'linewidth', 0.05);
plot(t, x_des(:,2), 'b-.', 'linewidth', 0.05);

plot(t, x(:,3), 'r--', 'linewidth', 1);
plot(t, error(:,3), 'g--', 'linewidth', 1);
plot(t, x_des(:,3), 'b--', 'linewidth', 1);
hold off;
grid on;
legend('State $x$ ',...
       'Error $e$',...
       'Desired $x_d$');
xlabel('Time $t$ [s]');
ylabel('State $x$');

% Subplot controller activity
subplot(2,1,2);
plot(t, u, 'linewidth', 1);
grid on;
legendCell = cellstr(num2str([1:3]', 'Control $u_%-d$'));
legend(legendCell);
xlabel('Time $t$ [s]');
ylabel('Control $u$');
% Save plot as .png file

Control = ['P(1,1)_',num2str(k1(1,1)), '_P(2,2)_',num2str(k1(2,2)), '_P(3,3)_', num2str(k1(3,3)), ...
    '_D(1,1)_', num2str(k2(1,1)), '_D(2,2)_', num2str(k2(2,2)),'_D(3,3)_', num2str(k2(3,3))];
Type_Control = 'Robust_Certain';
Type_Traj = 'PTP';

Photo = [Type_Control,'_Control_',Type_Traj,'_', join(Control),'.png'];
saveas(gcf,join(Photo))


