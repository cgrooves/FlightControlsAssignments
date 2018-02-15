P = struct;
P.g = 9.81;
   
% Using Aerosonde UAV parameters, Appendix E.
% physical parameters of airframe
P.mass = 13.5;
P.Jx   = 0.8244;
P.Jy   = 1.135;
P.Jz   = 1.759;
P.Jxz  = 0.1204;

% aerodynamic coefficients
P.S_wing        = 0.55;
P.b             = 2.8956;
P.c             = 0.18994;
P.S_prop        = 0.2027;
P.rho           = 1.2682;
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;
P.AR            = P.b^2/P.S_wing;

P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1.0;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;

% Gamma values
P.Gamma = P.Jx*P.Jz - P.Jxz^2;

P.Gamma1 = P.Jxz*(P.Jx - P.Jy + P.Jz)/P.Gamma;
P.Gamma2 = (P.Jz*(P.Jz - P.Jy) + P.Jxz^2)/P.Gamma;
P.Gamma3 = P.Jz/P.Gamma;
P.Gamma4 = P.Jxz/P.Gamma;
P.Gamma5 = (P.Jz - P.Jx)/P.Jy;
P.Gamma6 = P.Jxz/P.Jy;
P.Gamma7 = ((P.Jx - P.Jy)*P.Jx + P.Jxz^2)/P.Gamma;
P.Gamma8 = P.Jx/P.Gamma;

% initial conditions
P.Va0 = 17;

P.pn0    =  -1000; % initial North position
P.pe0    =  0; % initial East position
P.pd0    =  0; % initial Down position (negative altitude)
P.u0     =  P.Va0; % initial velocity along body x-axis
P.v0     =  0; % initial velocity along body y-axis
P.w0     =  0; % initial velocity along body z-axis
P.phi0   =  0; % initial roll angle
P.theta0 =  0; % initial pitch angle
P.psi0   =  0; % initial yaw angle
P.p0     =  0; % initial body frame roll rate
P.q0     =  0; % initial body frame pitch rate
P.r0     =  0; % initial body frame yaw rate

% wind parameters
P.wind_n = 0;%3;
P.wind_e = 0;%2;
P.wind_d = 0;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1.06; 
P.sigma_v = 1.06;
P.sigma_w = .7;

P.Ts = 0.1;

%%%%%%%%%%%%%%%%%%%%%%%%
% Trim conditions
%%%%%%%%%%%%%%%%%%%%%%%%
P.Va0 = 35; % initial airspeed
gamma = 5*pi/180; % initial flight path angle
R = 250; % initial turn radius

% compute trim
[x_trim, u_trim] = compute_trim('sim_trim',P.Va0,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

% Set initial conditions to trim conditions
P.pn0 = 0;
P.pe0 = 0;
P.pd0 = 0;
P.u0 = x_trim(4);
P.v0 = x_trim(5);
P.w0 = x_trim(6);
P.phi0 = x_trim(7);
P.theta0 = x_trim(8);
P.psi0 = x_trim(9);
P.p0 = x_trim(10);
P.q0 = x_trim(11);
P.r0 = x_trim(12);

% Get linear transfer function models
% compute different transfer functions
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P);

% linearize the equations of motion around trim conditions
[A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);
