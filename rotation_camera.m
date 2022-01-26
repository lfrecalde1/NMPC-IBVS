function [R] = rotation_camera(h)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
x = h(1);
y = h(2);
z = h(3);
roll = 0;
pitch = 0;
yaw = h(6);

constante_z = -pi/2;
constante_y = pi;

rx = [1, 0, 0;...
    0, cos(roll), -sin(roll);...
    0, sin(roll), cos(roll)];

ry = [cos(pitch), 0, sin(pitch);...
    0, 1, 0;...
    -sin(pitch), 0, cos(pitch)];

rz = [cos(yaw), -sin(yaw), 0;...
    sin(yaw), cos(yaw), 0;...
    0, 0, 1];

ry1 =[cos(constante_y), 0, sin(constante_y);...
    0, 1, 0;...
    -sin(constante_y), 0, cos(constante_y)];

rz1 = [cos(constante_z), -sin(constante_z), 0;...
    sin(constante_z), cos(constante_z), 0;...
    0, 0, 1];

R = rz*ry*rx*ry1*rz1;
end

