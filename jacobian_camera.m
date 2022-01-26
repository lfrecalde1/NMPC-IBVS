function [J] = jacobian_camera(param, pixel, z)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
%% obtencion parametros camara
fx = param(1);
fy = param(2);
u0 = param(3);
v0 = param(4);


U_m = pixel;
%% Variables auxiliares para creacion del jacobiano de la imagen
aux=1;
J = [];

for k=1:2:length(pixel)
    ii=aux;
    i = k;
    j = k+1;
    
    
    J_manual = [-fx/(z(ii,1)), 0, U_m(i)/z(ii,1), U_m(i)*U_m(j)/(fx), -(fx^2+U_m(i)^2)/(fx), U_m(j);...
        0, -(fy)/z(ii,1), U_m(j)/z(ii,1), (fy^2+U_m(j)^2)/(fy), -(U_m(i)*U_m(j))/(fy), -U_m(i)];
    
    
    J =[J;J_manual];
    
    aux=aux+1;
end
end