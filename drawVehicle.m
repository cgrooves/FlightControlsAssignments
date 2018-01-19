%--------------------------------------------------------------
% FILE: drawVehicle.m
% AUTHOR: Caleb Groves
% DATE: 1/18/18
% 
% PURPOSE: Flight controls project 2 function, to be called from Simulink
% diagram, that draws a flight vehicle to the screen using vertices and
% faces.
%
%
% INPUT: Vector uu contains 12 state variables of air vehicle: pn, pe, pd,
% u, v, w, psi, theta, phi, p, q, r, plus time t.
%
%
% OUTPUT: Figure drawn to the screen with 3D representation of aircraft, 
% whose points are defined in the defineVehicleBody function.
%
%
% NOTES: 
%
%--------------------------------------------------------------

%--------------------------------------------------------------
% Function to be called from Simulink vehicle animation diagram. Sets up
% the figure and calls appropriate functions to draw the vehicle.
%--------------------------------------------------------------
function drawVehicle(uu)
    % inputs to function
    pn = uu(1);
    pe = uu(2);
    pd = uu(3);
    u = uu(4);
    v = uu(5);
    w = uu(6);
    psi = uu(7);
    theta = uu(8);
    phi = uu(9);
    p = uu(10);
    q = uu(11);
    r = uu(12);
    t = uu(13);
    
    % define persistent variables
    persistent vehicle_handle
    persistent Vertices
    persistent Faces
    persistent facecolors
    
    % for first time, initialize plot and vars
    if t==0
        figure(1), clf
        [Vertices,Faces,facecolors] = defineVehicleBody;
        vehicle_handle = drawVehicleBody(Vertices,Faces,facecolors,...
            pn,pe,pd,phi,theta,psi,...
            [],'normal');
        
        title('Vehicle')
        xlabel('East')
        ylabel('North')
        zlabel('Up')
        view(32,47)
        axis([-10,10,-10,10,-10,10]);
        hold on
        
    else
        drawVehicleBody(Vertices,Faces,facecolors,...
            pn,pe,pd,phi,theta,psi,...
            vehicle_handle);
    end
end

%--------------------------------------------------------------
% Transforms vehicle body by input rotations and translations, and puts
% it into MATLAB plotting space.
%--------------------------------------------------------------
function handle = drawVehicleBody(V,F,patchcolors,...
    pn,pe,pd,phi,theta,psi,...
    handle,mode)

    V = rotate(V,phi,theta,psi); % rotate vehicle
    V = translate(V,pn,pe,pd); % translate vehicle

    % Transform vertices from NED to XYZ
    R = [0 1 0;
        1 0 0;
        0 0 -1];
    V = R*V;
    
    if isempty(handle)
        handle = patch('Vertices',V','Faces',F,...
            'FaceVertexCData',patchcolors,...
            'FaceColor','flat',...
            'EraseMode',mode);
        
    else
        set(handle,'Vertices',V','Faces',F);
        drawnow
    end
            
end

%--------------------------------------------------------------
% Translates pts by x,y,z and returns XYZ
%--------------------------------------------------------------
function XYZ = translate(pts,x,y,z)

    XYZ = pts + repmat([pn;pe;pd],1,size(pts,2));

end

%--------------------------------------------------------------
% Rotates pts by Euler angles yaw, pitch and roll
%--------------------------------------------------------------
function XYZ = rotate(pts,psi,theta,phi)

    % define rotation matrix (right handed)
    R_roll = [...
          1, 0, 0;...
          0, cos(phi), sin(phi);...
          0, -sin(phi), cos(phi)];
    R_pitch = [...
          cos(theta), 0, -sin(theta);...
          0, 1, 0;...
          sin(theta), 0, cos(theta)];
    R_yaw = [...
          cos(psi), sin(psi), 0;...
          -sin(psi), cos(psi), 0;...
          0, 0, 1];
    R = R_roll*R_pitch*R_yaw;  
    % note that R above either leaves the vector alone or rotates
    % a vector in a left handed rotation.  We want to rotate all
    % points in a right handed rotation, so we must transpose
    R = R';

    % rotate vertices
    XYZ = R*pts;
    
end

%--------------------------------------------------------------
% Defines vertices, faces, and colors for aircraft body
%--------------------------------------------------------------
function [V,F,facecolors] = defineVehicleBody
    
    % define frame variables
    fuse_h = 0.25;
    fuse_l1 = 0.5;
    fuse_l2;
    wing_l;
    wing_w;
    fuse_l3;
    fuse_w;
    tailwing_l;
    tailwing_w;
    tail_h;

    % Define the vertices
    V = [...
        fuse_l1, 0, 0.1*fuse_h; % pt 1
        fuse_l2, fuse_w/2, -0.4*fuse_h; % pt 2
        fuse_l2, -fuse_w/2, -0.4*fuse_h; % pt 3
        fuse_l2, -fuse_w/2, 0.4*fuse_h; % pt 4
        fuse_l2, fuse_w/2, 0.4*fuse_h; % pt 5
        -fuse_l3, 0, 0; % pt 6
        0, wing_w/2, 0; % pt 7
        -wing_l, wing_w/2, 0; % pt 8
        -wing_l, -wing_w/2, 0; % pt 9
        0, -wing_w/2, 0; % pt 10
        tailwing_l-fuse_l3, tailwing_w/2, 0; % pt 11
        -fuse_l3, tailwing_w/2, 0; % pt 12
        -fuse_l3, -tailwing_w/2, 0; % pt 13
        tailwing_l-fuse_l3, -tailwing_w/2, 0; % pt 14
        tailwing_l-fuse_l3, 0, 0; % pt 15
        -fuse_l3, 0, -tail_h; % pt 16
        ];
    
    % define the faces
    F = [...
        1, 2, 3; % fuselage - nose
        1, 3, 4;
        1, 4, 5;
        1, 2, 5;
        2, 3, 6; % fuselage - body
        3, 4, 6;
        4, 5, 6;
        5, 2, 6;
        7, 9, 8; % wings
        7, 10, 9;
        11, 13, 12; % tailwing
        11, 14, 13;
        15, 16, 6; % tail
        ];
    
    % define colors to be used
    nose = [1, 1, 0]; % yellow
    body = [1, 0, 0]; % red
    wings = [0, 1, 1]; % cyan
    tailwing = [0, 1, 1]; % cyan
    tail = [0, 0, 1]; % blue
        
    % assign colors to faces  
    facecolors = [...
        nose;
        nose;
        nose;
        nose;
        body;
        body;
        body;
        body;
        wings;
        wings;
        tailwing;
        tailwing;
        tail;
        ];        
    
end