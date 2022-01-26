function [v0, UB, LB] = optimization_parameters(N, v_mx)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here

%% Horizonte de prediccion
ul_mx = v_mx(1);
um_mx = v_mx(2);
un_mx = v_mx(3);
w_mx = v_mx(4);

v0 = ones(length(v_mx), N);

%% Generacion de las condicones del problema
lb = [-ul_mx, -um_mx, -un_mx, -w_mx]';
ub = [ ul_mx, um_mx, un_mx, w_mx]';
LB=[];
UB =[];
for index=1:N
    LB=[LB;lb];
    UB=[UB;ub];
end

end

