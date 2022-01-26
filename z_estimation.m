function [z] = z_estimation(camera, obj)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
object = reshape(obj, 3, size(obj,1)/3);

X = [object;...
     ones(1, size(obj,1)/3)];

%% Generacion de la ubicacion de la camara
h = camera(1:3,1);


R = rotation_camera(camera);

%% Matriz de tranformacion
H_e =[inv(R),-inv(R)*h;...
      0, 0, 0 ,1];
  
H = H_e*X;

z = H(3,:)';
end

