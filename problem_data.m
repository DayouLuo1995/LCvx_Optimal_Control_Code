% Copyright (c) 2021, Autonomous Controls Laboratory, University of Washington, Seattle
% -------------------------------------------------------------------------------------
% 3DoF Powered-Descent Guidance on Mars with lossless convexification for thrust lower 
% bound and thrust pointing constraints
% Coordinate System: (X-Y-Z) -> (East-North-Up)
% -------------------------------------------------------------------------------------

% clear all
% close all
% clc

% rng('default');

% N                       = 40;                                   % time-of-flight [s]
% dt                      = 1.5;                                  % sampling time [s]

% N                       = 30;                                   % time-of-flight [s]
% dt                      = 2.0;                                  % sampling time [s]

N                       = 30;                                   % time-of-flight [s]
dt                      = 3.0;                                  % sampling time [s]

alf                     = 0.0005;                               % thrust-specific fuel consumption (TSFC) [s/m]
min_throttle            = 0.2;                                  % minimum throttle 
max_throttle            = 0.8;                                  % maximum throttle
T_max                   = 24000;                                % maximum thrust magnitude [N]
vmax                    = 150;                                  % maximum velocity magnitude [m/s]
gamma_gs                = 5;                                    % minimum glide-slope angle [deg] 
theta_tp                = 50;                                   % maximum thrust vector pointing angle wrt -ve Z axis [deg]
mwet                    = 1905;                                 % wet mass of vehicle [kg] 
mdry                    = 1505;                                 % dry mass of vehicle [kg]
r_init                  = [0;2000;1500];                        % initial position in inertial frame [m]
v_init                  = [0;100;-75];                          % initial velocity in inertial frame [m/s]
r_fin                   = [0;0;0];                              % target position in inertial frame [m]
v_fin                   = [0;0;0];                              % target position in inertial frame [m/s]
g                       = [0;0;-3.71];                          % acceleration due to gravity in inertial frame on Mars surface [m/s^2]
rstrt                   = 1200;                                 % PIPG restarting parameter

prsr_name               = 'YALMIP';                             % parser choice 
slver_name              = 'MOSEK';                              % solver choice {CPLEX,CONEPROG,ECOS,GUROBI,MOSEK}

% choice of cost function {min-fuel,lqr} %%% min-effort and lqr currently unsupported
obj_flag                = 'lqr';
% obj_flag                = 'min-fuel';


% linear scaling
switch obj_flag
    case {'lqr'}

        ar                      = norm(r_init,"inf");
        av                      = norm(v_init,"inf");
        az                      = 0.07;
        au                      = 6;

        kmax                    = 7500;                                 % PIPG max iterations
    case 'min-fuel'
        ar                      = 2.65*norm(r_init,"inf");
        av                      = 1.15*norm(v_init,"inf");
        az                      = 0.076;
        au                      = 9.20;        

        kmax                    = 7500;                                 % PIPG max iterations
end

asig                    = au;

