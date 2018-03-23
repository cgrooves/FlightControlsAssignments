% path follow
%  - follow straight line path or orbit
%
% Modified:
%   3/25/2010  - RB
%   6/5/2010   - RB
%   11/08/2010 - RB
%   14/11/2014 - RWB
%
% input is:
%   flag - if flag==1, follow waypoint path
%          if flag==2, follow orbit
%   
%   Va^d   - desired airspeed
%   r      - inertial position of start of waypoint path
%   q      - unit vector that defines inertial direction of waypoint path
%   c      - center of orbit
%   rho    - radius of orbit
%   lambda - direction of orbit (+1 for CW, -1 for CCW)
%   xhat   - estimated MAV states (pn, pe, pd, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi)
%
% output is:
%  Va_c - airspeed command
%  h_c  - altitude command
%  chi_c - heading command
%  phi_ff - feed forward roll command
%
function out = path_follow(in,P)
  
  NN = 0;
  flag      = in(1+NN);
  Va_d      = in(2+NN);
  r_path    = [in(3+NN); in(4+NN); in(5+NN)];
  q_path    = [in(6+NN); in(7+NN); in(8+NN)];
  c_orbit   = [in(9+NN); in(10+NN); in(11+NN)];
  rho_orbit = in(12+NN);
  lam_orbit = in(13+NN);
  NN = NN + 13;
  pn        = in(1+NN);
  pe        = in(2+NN);
  h         = in(3+NN);
  Va        = in(4+NN);
  % alpha   = in(5+NN);
  % beta    = in(6+NN);
  phi       = in(7+NN);
  theta     = in(8+NN);
  chi       = in(9+NN);
  % p       = in(10+NN);
  % q       = in(11+NN);
   r       = in(12+NN);
  % Vg      = in(13+NN);
  % wn      = in(14+NN);
  % we      = in(15+NN);
  % psi     = in(16+NN);
  NN = NN + 16;
  t         = in(1+NN);
  
  % separate out path directions
  qn = q_path(1);
  qe = q_path(2);
  qd = q_path(3);
  
  rn = r_path(1);
  re = r_path(2);
  rd = r_path(3);
  
  pd = -h;
  
  switch flag
      case 1 % follow straight line path specified by r and q
          
          
          % Define relative error vector
          ep = [pn - rn;...
              pe - re;...
              pd - rd];
          
          n = 1/norm([qe -qn 0]')*[qe -qn 0]';
          s = ep - (ep'*n)*n;
          
          % Algorithm 3 ******************************************
          % 1. Define h_c = h_d
          h_c = -rd + sqrt(s(1)^2 + s(2)^2)*(qd/sqrt(qn^2 + qe^2));
          % 2.         
          chi_q = atan2(qe,qn);
          % 3-5
          while chi_q - chi < -pi
              chi_q = chi_q + 2*pi;
          end
          % 6-8
          while chi_q - chi > pi
              chi_q = chi_q - 2*pi;
          end
          % 9. Define error to minimize
          ep_y = -sin(chi_q)*(pn - rn) + cos(chi_q)*(pe - re);
          % 10. course command
          chi_c = chi_q - P.chi_inf*2/pi*atan(P.k_path*ep_y);
          
          phi_ff = 0;
           
      case 2 % follow orbit specified by c, rho, lam
          % Enumerate orbit center
          cn = c_orbit(1);
          ce = c_orbit(2);
          cd = c_orbit(3);
          
          % Algorithm 4 **********************************
          % 1.
          h_c = -cd;
          % 2.
          d = sqrt((pn - cn)^2 + (pe - ce)^2);
          % 3.
          phi = atan2(pe - ce, pn - cn);
          % 4 - 6
          while phi - chi < -pi
              phi = phi + 2*pi;
          end
          % 7 - 9
          while phi - chi > pi
              phi = phi - 2*pi;
          end
          % 10
          chi_c = phi + lam_orbit*(pi/2 + atan(P.k_orbit*(d - rho_orbit)/rho_orbit));
          % Feedforward phi command - derived from coordinated turn
          % expression
          phi_ff = lam_orbit*atan(Va^2/(P.g*rho_orbit));
  end
  
  % command airspeed equal to desired airspeed
  Va_c = Va_d;
  
  % create output
  out = [Va_c; h_c; chi_c; phi_ff];
end


