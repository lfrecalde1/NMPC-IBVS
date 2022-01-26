function [hp] = func_drone(h, v, L)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%% DEFINITION OF THE DYNAMIC MODEL USE THE JACOBIAN OF THE SYSTEM
J = drone_jacobian(h, L);

hp = J*v;
end

