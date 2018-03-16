clc
    close all
    
    %Plotting script to see the results from the simulation
    %Each plot has true state, commanded state, and estimated state
    fig_num = 1;
    
    plot_pn = 1;
    plot_pe = 1;
    plot_h = 0;
    
    plot_Va = 0;
    plot_alpha = 0;
    plot_beta = 0;
    
    plot_phi = 0;
    plot_theta = 0;
    plot_chi = 1;
    
    plot_p = 0;
    plot_q = 0;
    plot_r = 0;
    
    plot_Vg = 0;
    
    plot_inputs = 0; %de,dt,da,dr

    %Format our data file
    data = load('state_data.mat');
    data = data.ans;
    [n,m] = size(data);
    data = data(2:n,:);
    
    pn          = data(1,:);             % North position (meters)
    pe          = data(2,:);             % East position (meters)
    h           = data(3,:);             % altitude (meters)
    u           = data(4,:);             % body velocity along x-axis (meters/s)
    v           = data(5,:);             % body velocity along y-axis (meters/s)
    w           = data(6,:);             % body velocity along z-axis (meters/s)
    phi         = 180/pi*data(7,:);      % roll angle (degrees)   
    theta       = 180/pi*data(8,:);      % pitch angle (degrees)
    psi         = data(9,:);             % yaw angle (degrees)
    p           = 180/pi*data(10,:);     % body angular rate along x-axis (degrees/s)
    q           = 180/pi*data(11,:);     % body angular rate along y-axis (degrees/s)
    r           = 180/pi*data(12,:);     % body angular rate along z-axis (degrees/s)
    Va          = data(13,:);            % airspeed (m/s)
    alpha       = 180/pi*data(14,:);     % angle of attack (degrees)
    beta        = 180/pi*data(15,:);     % side slip angle (degrees)
    wn          = data(16,:);            % wind in the North direction
    we          = data(17,:);            % wind in the East direction
    wd          = data(18,:);            % wind in the Down direction
    pn_c        = data(19,:);            % commanded North position (meters)
    pe_c        = data(20,:);            % commanded East position (meters)
    h_c         = -data(21,:);            % commanded altitude (meters)
    Va_c        = data(22,:);            % commanded airspeed (meters/s)
    alpha_c     = 180/pi*data(23,:);     % commanded angle of attack (degrees)
    beta_c      = 180/pi*data(24,:);     % commanded side slip angle (degrees)
    phi_c       = 180/pi*data(25,:);     % commanded roll angle (degrees)   
    theta_c     = 180/pi*data(26,:);     % commanded pitch angle (degrees)
    chi_c       = 180/pi*data(27,:);     % commanded course (degrees)
    p_c         = 180/pi*data(28,:);     % commanded body angular rate along x-axis (degrees/s)
    q_c         = 180/pi*data(29,:);     % commanded body angular rate along y-axis (degrees/s)
    r_c         = 180/pi*data(30,:);     % commanded body angular rate along z-axis (degrees/s)
    pn_hat      = data(31,:);            % estimated North position (meters)
    pe_hat      = data(32,:);            % estimated East position (meters)
    h_hat       = -data(33,:);            % estimated altitude (meters)
    Va_hat      = data(34,:);            % estimated airspeed (meters/s)
    alpha_hat   = 180/pi*data(35,:);     % estimated angle of attack (degrees)
    beta_hat    = 180/pi*data(36,:);     % estimated side slip angle (degrees)
    phi_hat     = 180/pi*data(37,:);     % estimated roll angle (degrees)   
    theta_hat   = 180/pi*data(38,:);     % estimated pitch angle (degrees)
    chi_hat     = 180/pi*data(39,:);     % estimated course (degrees)
    p_hat       = 180/pi*data(40,:);     % estimated body angular rate along x-axis (degrees/s)
    q_hat       = 180/pi*data(41,:);     % estimated body angular rate along y-axis (degrees/s)
    r_hat       = 180/pi*data(42,:);     % estimated body angular rate along z-axis (degrees/s)
    Vg_hat      = data(43,:);            % estimated groundspeed
%    wn_hat      = data(44,:);            % estimated North wind
%    we_hat      = data(45,:);            % estimated East wind
%    psi_hat     = 180/pi*data(46,:);     % estimated heading
%    bx_hat      = data(47,:);            % estimated x-gyro bias
%    by_hat      = data(48,:);            % estimated y-gyro bias
%    bz_hat      = data(49,:);            % estimated z-gyro bias
    delta_e     = 180/pi*data(50,:);     % elevator angle (degrees)
    delta_a     = 180/pi*data(51,:);     % aileron angle (degrees)
    delta_r     = 180/pi*data(52,:);     % rudder angle (degrees)
    delta_t     = data(53,:);            % throttle setting (unitless)
    t           = data(54,:); 
    
    chi = 180./pi.*atan2(Va.*sin(psi)+we, Va.*cos(psi)+wn);

    if plot_pn == 1
       figure(fig_num);
       plot(t,pn,'b',t,pn_hat,'g',t,pn_c,'r--'); 
       title('pn');
       fig_num = fig_num+1;
    end
    
     if plot_pe == 1
       figure(fig_num)
       plot(t,pe,'b',t,pe_hat,'g',t,pe_c,'r--'); 
       title('pe');
       fig_num = fig_num+1;
     end
 
     if plot_h == 1
       figure(fig_num)
       plot(t,h,'b',t,h_hat,'g',t,h_c,'r--'); 
       title('h');
       fig_num = fig_num+1;
     end
    
     if plot_Va == 1
        figure(fig_num)
        plot(t,Va,'b',t,Va_hat,'g',t,Va_c,'r--'); 
        title('Va');
        fig_num = fig_num+1;
     end
    
     if plot_alpha == 1
        figure(fig_num)
        plot(t,alpha,'b',t,alpha_hat,'g',t,alpha_c,'r--'); 
        title('alpha');
        fig_num = fig_num+1;
     end
    
     if plot_beta == 1
        figure(fig_num)
        plot(t,beta,'b',t,beta_hat,'g',t,beta_c,'r--'); 
        title('beta');
        fig_num = fig_num+1;
      end
    
      if plot_phi == 1
         figure(fig_num)
         plot(t,phi,'b',t,phi_hat,'g',t,phi_c,'r--'); 
         title('phi');
         fig_num = fig_num+1;
      end
    
      if plot_theta == 1
         figure(fig_num)
         plot(t,theta,'b',t,theta_hat,'g',t,theta_c,'r--'); 
         title('theta');
         fig_num = fig_num+1;
      end
    
      if plot_chi == 1
         figure(fig_num)
         plot(t,chi,'b',t,chi_hat,'g',t,chi_c,'r--'); 
         title('chi');
         fig_num = fig_num+1;
      end
    
      if plot_p == 1
         figure(fig_num);
         plot(t,p,'b',t,p_hat,'g',t,p_c,'r--'); 
         title('p');
         fig_num = fig_num+1;
      end
      
      if plot_q == 1
         figure(fig_num);
         plot(t,q,'b',t,q_hat,'g',t,q_c,'r--'); 
         title('q');
         fig_num = fig_num+1;
      end
    
      if plot_r == 1
         figure(fig_num);
         plot(t,r,'b',t,r_hat,'g',t,r_c,'r--'); 
         title('r');
         fig_num = fig_num+1;
      end
    
      if plot_inputs == 1
         figure(fig_num);
         plot(t,delta_a,'b',t,delta_e,'g',t,delta_r,'k'); 
         title('Control Surfaces');
         legend('delta a','delta e','delta r')
         fig_num = fig_num+1;
         figure(fig_num);
         plot(t,delta_t,'r');
         title('Throttle');
      end
      
      if plot_Vg == 1
         figure(fig_num);
         plot(t,Vg,'b',t,Vg_hat,'g'); 
         title('Vg');
         fig_num = fig_num+1;
      end