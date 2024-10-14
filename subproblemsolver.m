function [x, u,un, s, dt] = subproblemsolver(Ac, Bc, rho1, rho2, dt, T_s, N, x0,u_s,x_end, flag )
% Variable definitions
x = sdpvar(4,N+1);
u = sdpvar(2,N);
s = sdpvar(1,N);

% YALMIP setup
ops        = sdpsettings;
ops.solver = 'mosek';
ops.verbose = 1;

% Discretizing the Continuous System
csys = ss(Ac,Bc,[],[]);
dsys = c2d(csys,dt);
A = dsys.A;
B = dsys.B;
if(flag == true)
dsy_start = c2d(csys, T_s- N * dt);
A_start = dsy_start.A;
B_start = dsy_start.B;
%
x0_N = A_start*x0 + B_start * u_s;
else
x0_N = x0;
end

% Optimization Problem
obj = x(3,N+1)^2 + x(4,N+1)^2;
%obj = obj*1000;%only for over-relaxed problem,as the square distroy the
%accuracy
con = [x(:,1) == x0_N, x(1:2,N+1) == x_end];
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


