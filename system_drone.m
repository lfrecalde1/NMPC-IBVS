function [h] = system_drone(h, v, ts, L)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
hdot = func_drone(h, v, L);

h = h + hdot*ts;
end

