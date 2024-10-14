function [x, u, s, dt, u_s] = lcvx_overrelaxed(Ac, Bc, rho1, rho2, N, dt0, x0, x_end, u_s)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%This not the final verstion.
%The major problem we have here is how to choose critira for the binary search.  
%We should consider both x and u. This involved solving the boundary only
%problem.
%In this example, we cheated as we know the best x is [1,1,0,0]

dt_l = 0.7;%for bijection
dt_u = dt0;
T_s = N*dt0;
%u_s = [rho1;0];% random initial control
epsilon = 5*10^-5* rho1;%the numerical solver has error
time_tol = 10^-5;
[x, u,un, s, dt] = subproblemsolver(Ac, Bc, rho1, rho2, dt_u, T_s, N, x0,u_s,x_end, false);
x_final = x(3:4, N+1);
if(sum(un>=rho1- epsilon)>= size(x,1) + 1 ||norm(x_final)>epsilon)
    return
end

while(dt_u- dt_l>= time_tol)
    temp = (dt_u + dt_l)/2;
    [x, u,un, s, dt] = subproblemsolver(Ac, Bc, rho1, rho2, temp, T_s, N, x0,u_s,x_end,true);
    x_final = x(3:4, N+1);
    if (sum(un>=rho1)>= 0.5*N ||norm(x_final)>epsilon)% for a more stable code, one should monitor both X and un. 
        dt_l = temp;
    else
        dt_u = temp;
    end
end
    [x, u,un, s, dt] = subproblemsolver(Ac, Bc, rho1, rho2, dt_u, T_s, N, x0,u_s,x_end,true);