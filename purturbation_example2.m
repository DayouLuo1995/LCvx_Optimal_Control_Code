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

x0 = [0;0;0];
N  = 10;
dt = 0.5/N;


% Discretizing the Continuous System

A = diag([1.2,-2.2 ,1]);
B = [0.4;0.3;0.2];


% YALMIP setup
ops        = sdpsettings;
ops.solver = 'mosek';
ops.mosek.MSK_DPAR_INTPNT_CO_TOL_REL_GAP = 1e-15;  
ops.mosek.MSK_DPAR_INTPNT_TOL_INFEAS = 1e-15;   
% Variable definitions
x = sdpvar(3,N+1);
u = sdpvar(1,N);
s = sdpvar(1,N);


% Optimization Problem

obj = x(3,N+1);
con = [x(:,1) == x0, x(1,N+1) == .2, x(2,N+1) == 2];%1,1
con = [x(:,1) == x0, x(1,N+1) == 0.5, x(2,N+1) == 1];
for k = 1:N
    con = [con, x(:,k+1) == A*x(:,k) + B*u(:,k)];
    con = [con, norm( u(:,k) )^2 <= s(k)];
    con = [con, rho1 <= s(k) <= rho2];
    obj = obj + s(k);
end


% Solve and extract
sol = optimize(con,obj,ops);
x  = double(x);
u  = double(u);
un = vecnorm(u,2,1);
s  = double(s);



% 
% % Plots
% figure
% plot(x','linewidth',2), grid on
% %title('states'), legend('x1','x2','x3')
% xlabel('index k')
% 
% figure
% plot(un.^2,'linewidth',2), hold on
% plot(s,'linewidth',2), grid on
% plot([1 N],[rho1 rho1],'k--','linewidth',2), hold on
% %plot([1 N],[rho2 rho2],'k--','linewidth',2)
% %title('control magnitude'), legend('||u||^2','sigma','rho1','rho2')
% xlabel('index k')

% Plotting and Saving Figures

% Define the time vector for plotting
time_vec = linspace(0, 10, N+1);

% States Plot Before Perturbation
figure;
plot(time_vec, x', 'linewidth', 2);
grid on;
%title('States Before Perturbation', 'Interpreter', 'latex');
legend('$x_1$', '$x_2$', '$x_3$', 'Interpreter', 'latex', 'Location', 'northeast');
xlabel('Time Index $k$', 'Interpreter', 'latex');
%xlim([1 11]);
% Save as EPS
%saveas(gcf, 'state_before_perturbation.eps', 'epsc');

% Control Magnitude Plot Before Perturbation
figure;
stairs(un.^2, 'linewidth', 2); hold on;
stairs( s, 'linewidth', 2); grid on;
stairs([1 N], [rho1 rho1], 'k--', 'linewidth', 2); 
%title('Control Magnitude Before Perturbation', 'Interpreter', 'latex');
legend('$||u||^2$', '$\sigma$', '$\rho_{min}$', 'Interpreter', 'latex', 'Location', 'northeast');
xlabel('Time Index $k$', 'Interpreter', 'latex');
xlim([1 10]);
% Save as EPS
%saveas(gcf, 'control_before_perturbation.eps', 'epsc');

%After perturbation
A = A + diag([1,0, 0])*1e-7;


% YALMIP setup
ops        = sdpsettings;
ops.solver = 'mosek';
ops.mosek.MSK_DPAR_INTPNT_CO_TOL_REL_GAP = 1e-15;  
ops.mosek.MSK_DPAR_INTPNT_TOL_INFEAS = 1e-15;   
% Variable definitions
x = sdpvar(3,N+1);
u = sdpvar(1,N);
s = sdpvar(1,N);


% Optimization Problem

obj = x(3,N+1);
con = [x(:,1) == x0, x(1,N+1) == .2, x(2,N+1) == 2];%1,1
con = [x(:,1) == x0, x(1,N+1) == 0.5, x(2,N+1) == 1];
for k = 1:N
    con = [con, x(:,k+1) == A*x(:,k) + B*u(:,k)];
    con = [con, norm( u(:,k) )^2 <= s(k)];
    con = [con, rho1 <= s(k) <= rho2];
    obj = obj + s(k);
end


% Solve and extract
sol = optimize(con,obj,ops);
x  = double(x);
u  = double(u);
un = vecnorm(u,2,1);
s  = double(s);




% Plots
figure;
plot(time_vec, x', 'linewidth', 2);
grid on;
%title('States After Perturbation', 'Interpreter', 'latex');
legend('$x_1$', '$x_2$', '$x_3$', 'Interpreter', 'latex', 'Location', 'northeast');
xlabel('Time Index $k$', 'Interpreter', 'latex');
%xlim([1 11]);
% Save as EPS
%saveas(gcf, 'state_after_perturbation.eps', 'epsc');

% Control Magnitude Plot Before Perturbation
figure;
stairs(un.^2, 'linewidth', 2); hold on;
stairs( s, 'linewidth', 2); grid on;
stairs([1 N], [rho1 rho1], 'k--', 'linewidth', 2); 
%title('Control Magnitude After Perturbation', 'Interpreter', 'latex');
legend('$||u||^2$', '$\sigma$', '$\rho_{min}$', 'Interpreter', 'latex', 'Location', 'northeast');
xlabel('Time Index $k$', 'Interpreter', 'latex');
xlim([1 10]);
% Save as EPS
%saveas(gcf, 'control_after_perturbation.eps', 'epsc');
