function [J] = drone_jacobian(h, L)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
x = h(1);
y = h(2);
z = h(3);
yaw = h(4);

a = L(1);
b = L(2);
c = L(3);

j11 = cos(yaw);
j12 = -sin(yaw);
j13 = 0;
j14 = -(a*sin(yaw)+b*cos(yaw));

j21 = sin(yaw);
j22 = cos(yaw);
j23 = 0;
j24 = a*cos(yaw)-b*sin(yaw);

j31 = 0;
j32 = 0;
j33 = 1;
j34 = 0;


j41 = 0;
j42 = 0;
j43 = 0;
j44 = 0;


j51 = 0;
j52 = 0;
j53 = 0;
j54 = 0;


j61 = 0;
j62 = 0;
j63 = 0;
j64 = 1;


J = [j11, j12, j13, j14;...
     j21, j22, j23, j24;...
     j31, j32, j33, j34;...
     j41, j42, j43, j44;...
     j51, j52, j53, j54;...
     j61, j62, j63, j64];  
end

