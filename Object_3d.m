function position_f = Object_3d(obj, h)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Object camera frame
object = reshape(obj, 3, size(obj,1)/3);
object = [object;...
          ones(1, size(obj,1)/3)];

% Get Rotation Matrix 
R = rotation_camera(h);

% get tranlation vector 
T = h(1:3);

H = [R, T;...
     0, 0, 0,1];

% Projection objet to the general frame
position = H*object;

% Clean vector the last row ones
position = position(1:3,:);


%% Reconstruction generalized vector
position_f = reshape(position,size(obj,1),1);
end

