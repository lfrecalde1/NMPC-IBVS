function [p, z] = open_loop_complete(h0, v0, vc, ts, N, L, pixel0, obj, param, chi, z0)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
h(:,1) = h0;
p(:,1) = pixel0;
v(:,1) = [v0(1:3);v0(6)];
z(:,1) = z0;
for k=1:1:N
    v(:,k+1) = system_dynamic(chi, v(:,k), vc(:,k), ts);
    h(:,k+1) = system_drone(h(:,k), v(:,k+1), ts, L);
    z(:,k+1) = z_estimation(h(:,k+1), obj);
    p(:,k+1) = system_camera(h(:,k+1), v(:,k+1), p(:,k), z(:,k+1), param, L, ts);
end
end
