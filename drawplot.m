function [t,u, X]= drawplot( u_2, u_s,dt, dt0,x0,N, Ac, Bc, rho1)

T = N *dt0;
dts = (T - dt * N)/N;

sys = ss(Ac, Bc, [], []);
dsys1 = c2d(sys, dts, 'zoh');
dsys2 = c2d(sys, dt, 'zoh');

u_0 = repmat(u_s, 1, N);

x = x0;
X = repmat(x0, 1, 2*N+1);
u= [u_0, u_2];
for i = 1:2*N
    if i <=N
        x = dsys1.A*x + dsys1.B*u(:,i);
    else
        x = dsys2.A* x + dsys2.B*u(:,i);
    end
    X(:,i+1) = x;
end
%time
t1 = 0:dts:(N) * dts;
t2 = t1(end) + (dt:dt: (N* dt));
t = [t1,t2];



set(0, 'DefaultAxesFontSize', 14);
set(0, 'DefaultAxesFontName', 'Arial');
set(0, 'DefaultLineLineWidth', 2);

% % State Variables Plot
% figure;
% plot(t, X', 'linewidth', 2); % Increased line width for consistency
% grid on; % Enable grid
% title('State Variables');
% xlabel('Time');
% ylabel('State');
% legend('x_1', 'x_2', 'x_3', 'x_4');
figure;
plot(t, X', 'linewidth', 2); % Increased line width for consistency
grid on; % Enable grid
%title('Long Horizon State after Bisection', 'Interpreter', 'latex');
xlabel('Time', 'Interpreter', 'latex');

legend('$x_1$', '$x_2$', '$x_3$', '$x_4$', 'Interpreter', 'latex');
saveas(gcf, 'over_relaxed_state_after.eps', 'epsc');
% Control Variables Plot
% figure;
% u = [u_s, u];
% plot(t, u', 'linewidth', 2); % Increased line width for consistency
% grid on; % Enable grid
% title('Control Variables');
% xlabel('Time');
% ylabel('Control');
% legend('$u_1$', '$u_2$', 'Interpreter', 'latex');

% Norm of Control Input at Each Time Step
figure;
u_norms = vecnorm(u);
stairs(t(2:end), u_norms, 'linewidth', 2); % Plotting u_norms with increased line width
ylim([0, rho1*2]);
grid on; % Enable grid
%title('Long Horizon Control after Bisection','Interpreter', 'latex');

yline(rho1, 'r--', 'LineWidth', 2); % Added line width for yline and label for clarity
legend('$\|u\|$', '$\rho_{min} = 1$', 'Interpreter', 'latex');
saveas(gcf, 'over_relaxed_control_after.eps', 'epsc');