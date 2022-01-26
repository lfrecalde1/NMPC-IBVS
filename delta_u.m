function [du] = delta_u(u, N)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
du = [];
for k=1:N-1
    du = [du;u(:,k+1)-u(:,k)];
end
end

