function control = classic_controller(h, pixel, pixeld, z, param, L)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
R = rotation_camera(h);

Rotacion = [inv(R), zeros(3,3);
            zeros(3,3), inv(R)];
        
J=jacobian_camera(param, pixel, z);

J_movil = drone_jacobian(h, L);

aux = [1,0,0,0;...
       0,1,0,0;...
       0,0,1,0;...
       0,0,0,0;...
       0,0,0,0;...
       0,0,0,1];
   
J_total = J*Rotacion*J_movil;

K1 = 1*eye(8);
K2 = 10*eye(8);

he = (pixeld - pixel);

control = pinv(J_total)*(K2*tanh(inv(K2)*K1*he));

control =  control';
end