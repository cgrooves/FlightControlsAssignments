    % estimate_states
%   - estimate the MAV states using gyros, accels, pressure sensors, and
%   GPS.
%
% Outputs are:
%   pnhat    - estimated North position, 
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 
%   Vahat    - estimated airspeed, 
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle, 
%   thetahat - estimated pitch angel, 
%   chihat   - estimated course, 
%   phat     - estimated roll rate, 
%   qhat     - estimated pitch rate, 
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed, 
%   wnhat    - estimate of North wind, 
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
% 
% 
% Modified:  3/15/2010 - RB
%            5/18/2010 - RB
%

function xhat = estimate_states(uu, P)

   % rename inputs
   y_gyro_x      = uu(1);
   y_gyro_y      = uu(2);
   y_gyro_z      = uu(3);
   y_accel_x     = uu(4);
   y_accel_y     = uu(5);
   y_accel_z     = uu(6);
   y_static_pres = uu(7);
   y_diff_pres   = uu(8);
   y_gps_n       = uu(9);
   y_gps_e       = uu(10);
   y_gps_h       = uu(11);
   y_gps_Vg      = uu(12);
   y_gps_course  = uu(13);
   t             = uu(14);
   
   % Initialize values
  persistent y_static_pres_d1;
  persistent y_diff_pres_d1;
  persistent y_gyro_x_d1;
  persistent y_gyro_y_d1;
  persistent y_gyro_z_d1;
   
   if t == 0
       y_static_pres_d1 = 0;
       y_diff_pres_d1 = 0;
       y_gyro_x_d1 = 0;
       y_gyro_y_d1 = 0;
       y_gyro_z_d1 = 0;
   end
   
   % Estimate angular rates   
   lpf_static_pres = LPF(y_static_pres_d1,y_static_pres,P.alpha_static_pres);
   lpf_diff_pres = LPF(y_diff_pres_d1,y_diff_pres,P.alpha_diff_pres);
   
   hhat = lpf_static_pres/(P.rho*P.g);
   Vahat = sqrt(2*lpf_diff_pres/P.rho);
%    phat = LPF(y_gyro_x_d1,y_gyro_x,P.alpha_gyro_x);
%    qhat = LPF(y_gyro_y_d1,y_gyro_y,P.alpha_gyro_y);
%    rhat = LPF(y_gyro_z_d1,y_gyro_z,P.alpha_gyro_z);
    phat = y_gyro_x;
    qhat = y_gyro_y;
    rhat = y_gyro_z;
   
   % update old values
   y_static_pres_d1 = lpf_static_pres;
   y_diff_pres_d1 = lpf_diff_pres;
   y_gyro_x_d1 = phat;
   y_gyro_y_d1 = qhat;
   y_gyro_z_d1 = rhat;
   
   % Implement Continuous Discrete Extended Kalman Filter for pitch and
   % roll angles
   persistent xhat_a;
   persistent Pa;
   persistent Qa;
   persistent Ra;
   
   if t == 0
       xhat_a = [0, 0]';
       Pa = diag([0.01 0]);
       Qa = diag([1e-11, 1e-20]); %diag([(.005*pi/180)^2, (.005*pi/180)^2]);
       Ra = diag([.0025^2, .0025^2, .0025^2]);
   end
   
   % Prediction Steps
   N = 10;
   
   for i = 1:N
       phihat = xhat_a(1);
       thetahat = xhat_a(2);
       
       f = [...
           phat + qhat*sin(phihat)*tan(thetahat) + rhat*cos(phihat)*...
           tan(thetahat);...
           qhat*cos(phihat) - rhat*sin(phihat);...
           ];
       xhat_a = xhat_a + P.Ts/N*f;
       
       phihat = xhat_a(1);
       thetahat = xhat_a(2);
       
       Aa = [qhat*cos(phihat)*tan(thetahat) - rhat*sin(phihat)*tan(thetahat),...
           (qhat*sin(phihat) + rhat*cos(phihat))/cos(thetahat)^2;...
           -qhat*sin(phihat) - rhat*cos(phihat),    0];
       Pa = Pa + (P.Ts/N)*(Aa*Pa + Pa*Aa' + Qa);
   end

   % If measurement is received from sensor (every P.Ts or 100 Hz)
   if ~mod(t,.01)
%     if 0
       % Jacobian of h from x
       C = [...
           0, qhat*Vahat*cos(thetahat) + P.g*cos(thetahat);
           
           -P.g*cos(thetahat)*cos(phihat), ...
           -rhat*Vahat*sin(thetahat) - phat*Vahat*cos(thetahat) + ...
           P.g*sin(thetahat)*sin(phihat);
           
           P.g*cos(thetahat)*sin(phihat), ...
           qhat*Vahat*sin(thetahat) + P.g*sin(thetahat)*cos(phihat);...
           ];
       % update L gains
       L = Pa*C'*inv(Ra + C*Pa*C');
       
       % update P matrix - estimation error covariance matrix
       Pa = (eye(2) - L*C)*Pa;
       
       % use dynamic observer model for state estimation
       y = [y_accel_x, y_accel_y, y_accel_z]';
       h = [...
           qhat*Vahat*sin(thetahat) + P.g*sin(thetahat);
           
           rhat*Vahat*cos(thetahat) - phat*Vahat*sin(thetahat) - ...
           P.g*cos(thetahat)*sin(phihat);
           
           -qhat*Vahat*cos(thetahat) - P.g*cos(thetahat)*cos(phihat);...
           ];
       
       xhat_a = xhat_a + L*(y - h);
   end
   
   % extract out states
   phihat = xhat_a(1);
   thetahat = xhat_a(2);
   
   % Implement Cont.-Discrete EKF for pn, pe, Vg, Chi, wn, we, and psi
   persistent xhat_p;
   persistent Pp;
   persistent Qp;
   persistent Rp;
   
   % Initalize
   if t == 0
       xhat_p = [-1000; 0; P.Va0; P.psi0; 0; 0; P.psi0]';
       Pp = diag([10, 10, .025, (20*pi/180)^2, 5, 1, (8*pi/180)^2]);
       Rp = diag([10^2, 10^2, .05^2, (.45)^2, 25, 25]); % check stuff
       Qp = diag([100, 100, 10, 100, 9, 8, 8]);
   end
   
   % Prediction Steps
   for i = 1:N
       pn = xhat_p(1);
       pe = xhat_p(2);
       Vg = xhat_p(3);
       chi = xhat_p(4);
       wn = xhat_p(5);
       we = xhat_p(6);
       psi = xhat_p(7);
       
       psidot = qhat*sin(phihat)/cos(thetahat) + rhat*cos(phihat)/cos(thetahat);
       
       f = [...
           Vg*cos(chi);...
           
           Vg*sin(chi);...
           
           1/Vg*((Vahat*cos(psi) + wn)*(-Vahat*psidot*sin(psi)) + ...
           (Vahat*sin(psi) + we)*(Vahat*psidot*cos(psi)));...
           
           P.g/Vg*tan(phihat)*cos(chi-psi);...
           
           0;...
           0;...
           
           psidot;...
           ];
       
       xhat_p = xhat_p + (P.Ts/N)*f;
       
       pn = xhat_p(1);
       pe = xhat_p(2);
       Vg = xhat_p(3);
       chi = xhat_p(4);
       wn = xhat_p(5);
       we = xhat_p(6);
       psi = xhat_p(7);
       
       psidot = qhat*sin(phihat)/cos(thetahat) + rhat*cos(phihat)/cos(thetahat);
       
       A = [...
           0, 0, cos(chi), -Vg*sin(chi), 0, 0, 0;...
           
           0, 0, sin(chi), Vg*cos(chi), 0, 0, 0;...
           
           0, 0, -f(3)/Vg, 0, -psidot*Vahat*sin(psi), psidot*Vahat*cos(psi), ...
           -psidot*Vahat*(wn*cos(psi) + we*sin(psi))/Vg;...
           
           0, 0, -P.g/Vg^2*tan(phihat)*cos(chi-psi), -P.g/Vg*tan(phihat)*...
           sin(chi-psi), 0, 0, P.g/Vg*tan(phihat)*sin(chi-psi);...
           
           0, 0, 0, 0, 0, 0, 0;...
           
           0, 0, 0, 0, 0, 0, 0;...
           
           0, 0, 0, 0, 0, 0, 0;...
           ];
       
       Pp = Pp + (P.Ts/N)*(A*Pp + Pp*A' + Qp);
   end
   
   % Sensor measurement update
   if ~mod(t,.1)
       
       C = [...
           1, 0, 0, 0, 0, 0, 0;...
           
           0, 1, 0, 0, 0, 0, 0;...
           
           0, 0, 1, 0, 0, 0, 0;...
           
           0, 0, 0, 1, 0, 0, 0;...
           
           0, 0, -cos(chi), Vg*sin(chi), 1, 0, -Vahat*sin(psi);...
           
           0, 0, -sin(chi), -Vg*cos(chi), 0, 1, Vahat*cos(psi);...
           ];
       
       y = [y_gps_n;...
           y_gps_e;...
           y_gps_Vg;...
           y_gps_course;...
           Vahat*cos(psi) + wn - Vg*cos(chi);...
           Vahat*sin(psi) + we - Vg*sin(chi);...
           ];
       
       h = [...
           pn;
           pe;
           Vg;
           chi;
           Vahat*cos(psi) + wn - Vg*cos(chi);...
           Vahat*sin(psi) + we - Vg*sin(chi);...
           ];
       
       L = Pp*C'*inv(Rp + C*Pp*C');
       
       Pp = (eye(7) - L*C)*Pp;
       
       xhat_p = xhat_p + L*(y - h);
   end
   
    pnhat = xhat_p(1);
    pehat = xhat_p(2);
    Vghat = xhat_p(3);
    chihat = xhat_p(4);
    wnhat = xhat_p(5);
    wehat = xhat_p(6);
    psihat = xhat_p(7);
  
    % not estimating these states 
    alphahat = 0;
    betahat  = 0;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    
      xhat = [...
        pnhat;...
        pehat;...
        hhat;...
        Vahat;...
        alphahat;...
        betahat;...
        phihat;...
        thetahat;...
        chihat;...
        phat;...
        qhat;...
        rhat;...
        Vghat;...
        wnhat;...
        wehat;...
        psihat;...
        bxhat;...
        byhat;...
        bzhat;...
        ];
end

% Low-pass filter function: pass in previous filtered value, new 
% measurement, and filter gain value, get out new filtered value; old value
% is updated to new one.
function y = LPF(y_d1, u, alpha)

y = alpha*y_d1 + (1-alpha)*u;

end