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
% u, v, w, psi, theta, phi, p, q, r.
%
%
% OUTPUT: Figure drawn to the screen with 3D representation of aircraft, 
% whose points are defined in the defineVehicleBody function.
%
%
% NOTES: 
%
%--------------------------------------------------------------

% Function to be called from Simulink vehicle animation diagram. Sets up
% the figure and calls appropriate functions to draw the vehicle.
function drawVehicle(uu)

end

% Transforms vehicle body by input rotations and translations, and puts
% it into MATLAB plotting space.
function handle = drawVehicleBody(V,F,patchcolors,...
    pn,pe,pd,phi,theta,psi,...
    handle,mode)

end

% Translates pts by x,y,z and returns XYZ
function XYZ = translate(pts,x,y,z)

end

% Rotates pts by Euler angles yaw, pitch and roll
function XYZ = rotate(pts,psi,theta,phi)

end

% Defines vertices, faces, and colors for aircraft body
function [V,F,facecolors] = defineVehicleBody

end