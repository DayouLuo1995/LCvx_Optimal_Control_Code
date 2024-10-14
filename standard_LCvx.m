% -------------------------------------------------------------------------
% MATLAB Code for "Revisiting Lossless Convexification: Theoretical Guarantees
% for Discrete-time Optimal Control Problems"
% Author: Dayou Luo, Kazuya Echigo, and Behçet Açıkmeşe
% Date: [10/14/2024]
% 
% This code implements the methods described in the paper:
% "Revisiting Lossless Convexification: Theoretical Guarantees for 
% Discrete-time Optimal Control Problems."
% 
% 
% Dependencies:
% - MATLAB 2024 or later
% - MoseK
% - Yamlip
% Usage:
% Run the scripts for corresponding examples
%
% We have three different examples
% - Standard_LCvx as example 1
% - Perturbation as example 2
% - Long horizon as example 3
% 
% Reference:
% Dayou Luo, Kazuya Echigo, Behçet Açıkmeşe, "Revisiting Lossless Convexification:
% Theoretical Guarantees for Discrete-time Optimal Control Problems," [Journal Name], 2024.
% -------------------------------------------------------------------------


clear all; close all; clc

% Continuous Dynamics
rho1 = 1;
rho2 = 2;
I  = eye(2);
Z  = zeros(2,2);
Ac = [Z I; Z Z];
Bc = [Z; I];
x_end = [1;1];
N  = 10;
dt0 = 1;
u_s = [rho1;0];
x0 = [1;2;3;4];
T_s = N*dt0;

[x, u,un, s, dt] = subproblemsolver(Ac, Bc, rho1, rho2, dt0, T_s, N, x0,u_s,x_end, false);
time_vec = linspace(0, N*dt0, N+1); % Adjust based on your actual time vector

% States Plot
figure;
plot(time_vec, x', 'linewidth', 2);
grid on;
%title('States', 'Interpreter', 'latex');
legend('$x_1$', '$x_2$', '$x_3$', '$x_4$', 'Interpreter', 'latex', 'Location', 'northeast');
xlabel('Time Index $k$', 'Interpreter', 'latex');
xlim([0, N]);
% Save as EPS
%saveas(gcf, 'state_standard.eps', 'epsc');
% 
% Control Magnitude Plot
rho_min = 1;
rho_max = 2;
figure;
time_vec = linspace(1, N*dt0, N); 
stairs(un.^2, 'linewidth', 2); hold on;
stairs( s, 'linewidth', 2); grid on;
stairs([1, N], [rho_min, rho_min], 'k--', 'linewidth', 2); hold on;
stairs([1, N], [rho_max, rho_max], 'k--', 'linewidth', 2);
%title('Control Magnitude', 'Interpreter', 'latex');
legend('$||u||^2$', '$\sigma$', '$\rho_{min}$', '$\rho_{max}$', 'Interpreter', 'latex', 'Location', 'northeast');
xlabel('Time Index $k$', 'Interpreter', 'latex');
xlim([1, N]);
% Save as EPS
%saveas(gcf, 'control_standard.eps', 'epsc');

% Control Magnitude Plot
rho_min = 1;
rho_max = 2;
figure;
time_vec = linspace(1, N*dt0, N);

% Red solid line for 'un.^2' with increased linewidth for visibility
stairs(un.^2, 'r', 'linewidth', 3); hold on;

% Blue dashed line for 's' with default linewidth
stairs(s, 'b--', 'linewidth', 2); grid on;

stairs([1, N], [rho_min, rho_min], 'k--', 'linewidth', 2); hold on;
stairs([1, N], [rho_max, rho_max], 'k--', 'linewidth', 2);

title('Control Magnitude', 'Interpreter', 'latex');
legend('$||u||^2$', '$\sigma$', '$\rho_{min}$', '$\rho_{max}$', 'Interpreter', 'latex', 'Location', 'northeast');
xlabel('Time Index $k$', 'Interpreter', 'latex');
xlim([1, N]);

% % Save as EPS
% saveas(gcf, 'control_standard.eps', 'epsc');
