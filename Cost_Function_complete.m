function [costo] = Cost_Function_complete(h0, v0, vc, ts, N, L, pixel0, hd, obj, param, chi, i, z0)
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here

[h, z] = open_loop_complete(h0, v0, vc, ts, N, L, pixel0, obj, param, chi, z0);

%% generacion del vector de estados deseados
hdaux = hd(:,i:i+(N));

%% Definimos el valor del costo inicial
[he, u ] = general(h, hdaux, vc, N);

du = delta_u(vc, N);

%% TRANFORMATION ERROR PIXELS TO METERS
fx = param(1);
fy = param(2);
uo = param(3);
vo = param(4);

Q = (1/(fx))*eye(length(he));
R = 10*eye(length(u));

%costo = he'*Q*he + du'*R1*du;
costo = he'*Q*he + u'*R*u;
end
