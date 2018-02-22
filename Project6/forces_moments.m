% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1); % elevator
    delta_a = delta(2); % aileron
    delta_r = delta(3); % rudder
    delta_t = delta(4); % throttle
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    % compute wind data in NED
    u_w = cos(theta)*cos(psi)*w_ns + cos(theta)*sin(psi)*w_es - sin(theta)*w_ds + u_wg;
    v_w = (sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*w_ns + (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*w_es + sin(phi)*cos(theta)*w_ds;
    w_d = (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*w_ns + (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*w_es + cos(phi)*cos(theta)*w_ds;
    
    % compute air data
    u_w = w_n + u_wg;
    v_w = w_e + v_wg;
    w_w = w_d + w_wg;
    
    u_r = u - u_w;
    v_r = v - v_w;
    w_r = w - w_w;
    
    % Compute airspeed, angle of attack, sideslip angle
    Va = sqrt(u_r^2 + v_r^2 + w_r^2);
    alpha = atan2(w_r,u_r);
    
    if Va ~= 0
        beta = asin(v_r / Va);
    else
        beta = 0;
        Va = .0001;
    end
    
%     beta = asin(v_r / Va);
    
    % nonlinear drag model
    C_D_alpha = P.C_D_p + (P.C_L_0 + P.C_L_alpha*alpha)^2/(pi*P.e*P.AR);
    
    % pitching moment
    C_m_alpha = P.C_m_0 + P.C_m_alpha*alpha;
    
    % Define C functions
    C_chi_alpha = -C_D_alpha*cos(alpha) + C_L(alpha,P)*sin(alpha);
    C_chi_q_alpha = -P.C_D_q*cos(alpha) + P.C_L_q*sin(alpha);
    C_chi_delta_e_alpha = -P.C_D_delta_e*cos(alpha) + P.C_L_delta_e*sin(alpha);
    C_Z_alpha = -C_D_alpha*sin(alpha) - C_L(alpha,P)*cos(alpha);
    C_Z_q_alpha = -P.C_D_q*sin(alpha) - P.C_L_q*cos(alpha);
    C_Z_de_alpha = -P.C_D_delta_e*sin(alpha) - P.C_L_delta_e*cos(alpha);
    
    % compute external forces and torques on aircraft
    Force(1) =  -P.mass*P.g*sin(theta) + P.rho*Va^2*P.S_wing/2*(C_chi_alpha + C_chi_q_alpha*P.c*q/2/Va + C_chi_delta_e_alpha*delta_e) + P.rho*P.S_prop*P.C_prop/2*((P.k_motor*delta_t)^2 - Va^2);
    Force(2) =  P.mass*P.g*cos(theta)*sin(phi) + P.rho*Va^2*P.S_wing/2*(P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_p*p*P.b/2/Va + P.C_Y_r*P.b*r/2/Va + P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r);
    Force(3) =  P.mass*P.g*cos(theta)*cos(phi) + P.rho*Va^2*P.S_wing/2*(C_Z_alpha + C_Z_q_alpha*P.c*q/2/Va + C_Z_de_alpha*delta_e);
    
    Torque(1) = P.rho*Va^2*P.S_wing/2*P.b*(P.C_ell_0 + P.C_ell_beta*beta + P.C_ell_p*P.b*p/2/Va + P.C_ell_r*P.b*r/2/Va + P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r) - P.k_T_P*(P.k_Omega*delta_t)^2;
    Torque(2) = P.rho*Va^2*P.S_wing/2*P.c*(P.C_m_0 + P.C_m_alpha*alpha + P.C_m_q*P.c*q/2/Va + P.C_m_delta_e*delta_e);   
    Torque(3) = P.rho*Va^2*P.S_wing/2*P.b*(P.C_n_0 + P.C_n_beta*beta + P.C_n_p*P.b*p/2/Va + P.C_n_r*P.b*r/2/Va + P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r);
   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end

% nonlinear lift model
function out = C_L(alpha,P)
    
% blending function
sigma = (1 + exp(-P.M*(alpha-P.alpha0)) + exp(P.M*(alpha+P.alpha0)))/...
    ((1 + exp(-P.M*(alpha-P.alpha0)))*(1 + exp(P.M*(alpha+P.alpha0))));

% nonlinear drag model
out = (1-sigma)*(P.C_L_0 + P.C_L_alpha*alpha) + sigma*(2*sign(alpha)*...
    sin(alpha)^2*cos(alpha));

end