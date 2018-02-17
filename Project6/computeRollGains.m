%% Roll Attitude Hold
% Compute roll attitude hold loop gains: kp_phi, kd_phi, ki_phi
delta_a_max = pi/4;
Va = 10; % m/s

%%%DESIGN PARAMETERS%%%
zeta_phi = 1.5;
phi_max = 15*pi/180;
%%%END%%%%%%%%%%%%%%%%%

a_phi1 = -P.rho*Va*P.S_wing*P.b^2*P.C_p_p/4;
a_phi2 = P.rho*Va^2*P.S_wing*P.b*P.C_p_delta_a/2;

kp_phi = delta_a_max/e_phi_max*sign(a_phi2);
wn_phi = sqrt(abs(a_phi2)*delta_a_max/phi_max);
kd_phi = (2*zeta_phi*wn_phi-a_phi1)/a_phi2;

rlocus(tf([a_phi2],[1,(a_phi1+a_phi2*kd_phi),a_phi2*kp_phi,0]))

ki_phi = 6.3;

%% Course Hold Gains
Vg = Va;

%%%DESIGN PARAMETERS%%%
zeta_chi = 0.7;
WX = 10;
%%%END%%%%%%%%%%%%%%%%%

wn_chi = wn_phi/WX; % bandwidth separation
kp_chi = 2*zeta_chi*wn_chi*Vg/P.g;
ki_chi = wn_chi^2*Vg/P.g;

%% Sideslip Hold Gains
delta_r_max = 45*pi/180;

%%%DESIGN PARAMETERS%%%%%%
e_beta_max = 5*pi/180;
zeta_beta = 0.7;
%%%END%%%%%%%%%%%%%%%%%%%%

a_beta1 = -P.rho*Va*P.S_wing/(2*P.mass)*P.C_Y_beta;
a_beta2 = P.rho*Va*P.S_wing/(2*P.mass)*P.C_Y_delta_r;

kp_beta = delta_r_max/e_beta_max*sign(a_beta2);
ki_beta = 1/a_beta2*((a_beta1 + a_beta2*kp_beta)/2/zeta_beta)^2;

%% Pitch Attitude Hold Gains
delta_e_max = 45*pi/180;

%%%DESIGN PARAMETERS%%%%%%
e_theta_max = 10*pi/180;
zeta_theta = .75;
%%%END%%%%%%%%%%%%%%%%%%%%

a_theta1 = -P.rho*Va^2*P.c*P.S_wing/(2*P.Jy)*P.C_m_q*P.c/(2*Va);
a_theta2 = -P.rho*Va^2*P.c*P.S_wing*P.C_m_alpha/(2*P.Jy);
a_theta3 = P.rho*Va^2*P.c*P.S_wing/(2*P.Jy)*P.C_m_delta_e;

kp_theta = delta_e_max/e_theta_max*sign(a_theta3);
wn_theta = sqrt(a_theta2 + kp_theta*a_theta3);
kd_theta = (2*zeta_theta*wn_theta - a_theta1)/a_theta3;

K_theta_DC = (kp_theta*a_theta3)/(a_theta2 + kp_theta*a_theta3);

%% Altitude Hold Gains

%%%DESIGN PARAMETERS%%%%%%
zeta_h = 0.8;
W_h = 15;
%%%END%%%%%%%%%%%%%%%%%%%%

wn_h = 1/W_h*wn_theta;

ki_h = wn_h^2/(K_theta_DC*Va);
kp_h = 2*zeta_h*wn_h/(K_theta_DC*Va);

%% Airspeed Hold with Pitch

%%%DESIGN PARAMETERS%%%%%%
W_V2 = 10;
zeta_V2 = 0.7;
%%%END%%%%%%%%%%%%%%%%%%%%

wn_V2 = wn_theta/W_V2;

a_V1 = P.rho*Va*P.S_wing/P.mass*(P.C_D_0 + P.C_D_alpha*alpha + ...
    P.C_D_delta_e*delta_e) + P.rho*P.S_wing*P.C_prop*Va/P.mass;
a_V2 = P.rho*P.S_prop*P.C_prop*P.k_motor^2*delta_t/P.mass;
a_V3 = P.g*cos(theta - chi);

ki_V2 = wn_V2^2/(K_theta_DC*P.g);
kp_V2 = (a_V1 - 2*zeta_V2*wn_V2)/(K_theta_DC*P.g);

%% Airspeed Hold with Throttle

%%%DESIGN PARAMETERS%%%%%%

%%%END%%%%%%%%%%%%%%%%%%%%