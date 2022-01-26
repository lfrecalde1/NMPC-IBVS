function [c,ceq] = constraints(h, v, param, z_c, zd_c)

f = param(1);
uo = param(2);
vo = param(3);
pw = param(4);

%% Inequality constrain  <=
% A = matrix_constrain(h);
% A1 = matrix_constrain(z_c);
% 
% z = max_min_constrian(h,[uo-50;vo-50]);
z1 = 1.32*ones(4,1);
%% Generacion de las contantes de restriccion


%% Restricciones  de estados del sistema
% c1 = A*h-z;

c2 = -z_c-z1;
c3 = z_c-zd_c;
%% crecion del vector final de restriccionos
c = [c2];

ceq  = [0];
end