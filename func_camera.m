function hp = func_camera(h, v, pixel, z, param, L)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
R = rotation_camera(h);

Rotacion = [inv(R), zeros(3,3);
            zeros(3,3), inv(R)];
        
J=jacobian_camera(param, pixel, z);

J_movil = drone_jacobian(h, L);

   
J_total = J*Rotacion*J_movil;
hp = J_total*v;
end