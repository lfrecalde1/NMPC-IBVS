function [c, ceq] = Nonlinear_constraint(h0, v0, vc, ts, N, L, pixel0, obj, param, chi, z0, zd)
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here

[h, z] = open_loop_complete(h0, v0, vc, ts, N, L, pixel0, obj, param, chi, z0);

%% Generacion de vector de estados de restricciones
c = [];
ceq = [];

for k=1:N
    [desigualdad, igualdad] = constraints(h(:,k), vc(:,k), param, z(:,k), zd);
    c = [c desigualdad];
    ceq = [ceq igualdad];
    
end
end