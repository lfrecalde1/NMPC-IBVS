function [uv_center, z, obj, flag] = image_data(pixel_data, camera_parameters)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
% Creation ob the object that recive the information
pixels = receive(pixel_data,3);

% Get values of the pixels and deep
u1 = double(pixels.U1);
v1 = double(pixels.V1);
x1 = pixels.X1;
y1 = pixels.Y1;
z1 = pixels.Z1;

u2 = double(pixels.U2);
v2 = double(pixels.V2);
x2 = pixels.X2;
y2 = pixels.Y2;
z2 = pixels.Z2;

u3 = double(pixels.U3);
v3 = double(pixels.V3);
x3 = pixels.X3;
y3 = pixels.Y3;
z3 = pixels.Z3;

u4 = double(pixels.U4);
v4 = double(pixels.V4);
x4 = pixels.X4;
y4 = pixels.Y4;
z4 = pixels.Z4;

% Generation of general vectors and flag for control
z = [z1, z2, z3, z4]';

obj = [x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4]';

uv_image = [u1, v1, u2, v2, u3, v3, u4, v4]';

flag = pixels.Flag;

uv_center = center_image(uv_image, camera_parameters);
end

