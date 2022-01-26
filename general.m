function [he,u] = general(h, hd, v, N)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
he = [];
u = [];
for k=1:N
   he = [he;hd(:,k)-h(:,k)];
   u = [u;v(:,k)];
end

%% Costo Final
% he = [he;hd(:,k+1)-h(1:2,k+1)];
end

