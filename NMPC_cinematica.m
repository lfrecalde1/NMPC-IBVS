function [H0, control] = NMPC_cinematica(p, obj_c, h, v, hd, k, H0, vc, args, solver ,N)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
args.p(1:25) = [p;obj_c;h(4);v];

for i = 1:N
    args.p(25*i+1:25*i+25)=[hd(:,k+i)];
end



args.x0 = [reshape(H0',25*(N+1),1);reshape(vc',size(vc,2)*N,1)]; % initial value of the optimization variables

sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
    'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);

control = reshape(full(sol.x(25*(N+1)+1:end))',4,N)';
H0 = reshape(full(sol.x(1:25*(N+1)))',25,N+1)';
end


