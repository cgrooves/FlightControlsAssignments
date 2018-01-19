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
    
    % Define the vertices
    
    
    % define the faces
    
    % define colors to be used
    
    % assign colors to faces   
    
end