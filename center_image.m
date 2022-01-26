function [pixel_center] = center_image(uv, camera_parameters)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
fx = camera_parameters(1);
fy = camera_parameters(2);
u0 = camera_parameters(3);
v0 = camera_parameters(4);


% TRANSFORM PIXEL GENRALIZED FORM TO TRANSFORMATION FORM
pixel = reshape(uv, 2, size(uv,1)/2);
pixel = [pixel;...
         ones(1,size(uv,1)/2)];
     
T = [1, 0, -u0;...
     0, 1, -v0;...
     0, 0, 1];
 
%% GENERATE NEW PIXELS RESPECTO TO CENTER OF IMAGE
center = T*pixel;
center = center(1:2,:);

%% PIXEL RESPECT TO CENTER IMAGE
pixel_center = reshape(center, size(uv,1), 1);
end

