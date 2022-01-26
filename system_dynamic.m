function [v] = system_dynamic(chi, v, vc, ts)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
vp = dynamic_func(chi, v, vc);
v = v +vp*ts;
end

