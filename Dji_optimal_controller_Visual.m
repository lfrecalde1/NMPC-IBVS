%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXX TRAJECTORY CONTROL DJI DRONE XXXXXXXXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

%% CLEAN VARIABLES
clc,clear all,close all;

%% DEFINITION OF TIME VARIABLES
ts = 0.1;
tf = 50;
to = 0;
t = (to:ts:tf);

%% ROS PARAMETER FOR COMUNICATION
rosshutdown
rosinit('192.168.1.2', 'NodeHost', '192.168.1.2', 'Nodename', '/Matlab_Visual_Servoing');

%% OBJECTS CREATION OF TOPICS ROS
robot = rospublisher('/Mavic_2_PRO/cmd_vel');
velmsg = rosmessage(robot);
odom = rossubscriber('/Mavic_2_PRO/odom');
pixel_data = rossubscriber('/Mavic_2_PRO/Pixels_data');

%% CONSTANTS VALUES OF THE ROBOT
a = 0.1; 
b = 0.1;
c = 0.0;
L = [a, b, c];

%% CONSTANT VALUES OF CAMERA
f0 = [6.7630115434170293e+03 * 0.0774; 6.8516194384907485e+03 * 0.0774];

uo = 2.4597651153505402e+02;
vo = 1.9955473351505680e+02;

param = [f0; uo; vo];

%% DIRECT KINEMATICS
%% READ VALUES FOR INITIAL CONDITIONS
[h, hp] = odometry(odom, L);
 
%% READ VALUES OF PIXELS
[uv, z, obj, flag] = image_data(pixel_data, param);
H = [uv(1:8,1);obj(1:12,1);h(6,1);[hp(1:3,1);hp(6,1)]];
% Projection obj general frame
obj_3d = Object_3d(obj, h);

%% ESTIMATION Z DEEP
z_esti = z_estimation(h, obj_3d);

%% definicion de los pixeles deseados del sistema
%% DESIRED PIXELS VALUES OF THE SYSTEM

uv_d = [-73.9765, -52.5547, 54.0235, -93.5547, 53.0235, 34.4453, -73.9765, -9.5547]';
uv_d = uv_d*ones(1,length(t));
obj_cd = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]';
obj_cd = obj_cd*ones(1,length(t));
theta_d = 0*ones(1,length(t));
v_d = [0 0 0 0]';
v_d = v_d*ones(1,length(t));

%% Vector generalizado de deseados del sistema
HD = [uv_d(1:8,:);obj_cd(1:12,:);theta_d;v_d];

N = 15;

%% Definicion de los limites de las acciondes de control
bounded = [1.0; -1.0; 1.0; -1.0; 1.0; -1.0; 1.5; -1.5];
%% Definicion del vectro de control inicial del sistema
vc = zeros(N,4);
H0 = repmat(H,1,N+1)';

%% LOAD DYAMIC PARAMETERS DRONE
load("parameters.mat");
[f, solver, args] = mpc_drone_cinematica(bounded, N, L, param, chi, ts);

% SIMULATION 
for k=1:1:length(t)-N
    tic; 
    %% GENERAL VECTOR OF ERROR SYSTEM
    he(:, k) = uv_d(:,k)-uv(:,k);
    
    %% GENERALIZED CONTROL LAW
%     f_obj1 = @(vc) Cost_Function_complete(h(:,k), hp(:,k), vc, ts, N, L, uv(:,k), uv_d, obj_3d(:,k), param, chi, k, z(:,k));
%     f_obj2 = @(vc) Nonlinear_constraint(h(:,k), hp(:,k), vc, ts, N, L, uv(:,k), obj_3d(:,k), param, chi, z(:,k), zd);
%     %control = fmincon(f_obj1,v0,[],[],[],[],LB,UB,f_obj2,options);
%     [H0, control] = NMPC_cinematica(uv(1:8,k), obj(1:12,k), [h(1:3,k);h(6,k)],[hp(1:3,k);hp(6,k)], HD, k, H0, vc, args, solver ,N);
     control_i = classic_controller(h(:,k), uv(:,k), uv_d(:,k), z(:,k), param, L);
  
     %control = [0;0;0;0];
    %% OBTAIN CONTROL VALUES OF THE VECTOR
    ul(k) = control_i(1,1);
    um(k) = control_i(1,2);
    un(k) = control_i(1,3);
    w(k) = control_i(1,4);
    
    %% SEND CONTROL VALUES TO THE ROBOT
    send_velocities(robot, velmsg, [ul(k), um(k), un(k), 0, 0 , w(k)], flag(1,k));
    
   
     %% READ STATES VALUES OF THE ROBOT
    [h(:,k+1), hp(:,k+1)] = odometry(odom, L);
    
    %% READ PIXELS AND DEEP VALUE
    [uv(:,k+1), z(:,k+1), obj(:,k+1), flag(1,k+1)] = image_data(pixel_data, param);
    
    
    obj_3d(:,k+1) = Object_3d(obj(:,k+1), h(:,k+1));
    %% UPDATE OPTIMAL VALUES
%     vc = [control(2:end,:);control(end,:)];
%     H0 = [H0(2:end,:);H0(end,:)];
   
    %v0 = horizonte(control);
    while(toc<ts)
    end
    toc;
    %% SAVE SAMPLE TIME
    t_sample(k)=toc;
end
send_velocities(robot, velmsg, [0, 0, 0, 0, 0 , 0], 1)
rosshutdown;
%%
close all; paso=1; 
%a) Parámetros del cuadro de animación
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 8 3]);
luz = light;
luz.Color=[0.65,0.65,0.65];
luz.Style = 'infinite';
%b) Dimenciones del Robot
   Drone_Parameters(0.02);
%c) Dibujo del Robot    
    G2=Drone_Plot_3D(h(1,1),h(2,1),h(3,1),0,0,h(4,1));hold on

    plot3(h(1,1),h(2,1),h(3,11),'--','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on
    plot3(obj_3d(1,1),obj_3d(2,1),obj_3d(3,1),'Color',[100,76,44]/255,'linewidth',1); hold on;
    plot3(obj_3d(4,1),obj_3d(5,1),obj_3d(6,1),'Color',[100,76,44]/255,'linewidth',1); hold on;
    plot3(obj_3d(7,1),obj_3d(8,1),obj_3d(9,1),'Color',[100,76,44]/255,'linewidth',1); hold on;
    plot3(obj_3d(10,1),obj_3d(11,1),obj_3d(12,1),'Color',[100,76,44]/255,'linewidth',1); hold on;

view(20,15);
for k = 1:10:length(t)-N
    drawnow
    delete(G2);
   
    G2=Drone_Plot_3D(h(1,k),h(2,k),h(3,k),0,0,h(4,k));hold on
    
    plot3(h(1,1:k),h(2,1:k),h(3,1:k),'--','Color',[56,171,217]/255,'linewidth',1.5);
    plot3(obj_3d(1,1:k),obj_3d(2,1:k),obj_3d(3,1:k),'Color',[100,76,44]/255,'linewidth',1); hold on;
    plot3(obj_3d(4,1:k),obj_3d(5,1:k),obj_3d(6,1:k),'Color',[100,76,44]/255,'linewidth',1); hold on;
    plot3(obj_3d(7,1:k),obj_3d(8,1:k),obj_3d(9,1:k),'Color',[100,76,44]/255,'linewidth',1); hold on;
    plot3(obj_3d(10,1:k),obj_3d(11,1:k),obj_3d(12,1:k),'Color',[100,76,44]/255,'linewidth',1); hold on;

    legend({'$\mathbf{h}$','$\mathbf{h}_{des}$'},'Interpreter','latex','FontSize',11,'Location','northwest','Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Movement Executed by the Aerial Robot}$','Interpreter','latex','FontSize',11);
    xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9);zlabel('$\textrm{Z}[m]$','Interpreter','latex','FontSize',9);
    
end
print -dpng SIMULATION_1
print -depsc SIMULATION_1


figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
set(gca,'Ydir','reverse')

plot(uv(1,:),uv(2,:),'--','Color',[223,67,85]/255,'linewidth',1); hold on
plot(uv(3,:),uv(4,:),'--','Color',[56,171,217]/255,'linewidth',1); hold on
plot(uv(5,:),uv(6,:),'--','Color',[46,188,89]/255,'linewidth',1); hold on
plot(uv(7,:),uv(8,:),'--','Color',[56,10,217]/255,'linewidth',1); hold on

plot(uv_d(1,1:length(uv)),uv_d(2,1:length(uv)),'x','Color',[223,67,85]/255,'linewidth',1); hold on
plot(uv_d(3,1:length(uv)),uv_d(4,1:length(uv)),'x','Color',[56,171,217]/255,'linewidth',1); hold on
plot(uv_d(5,1:length(uv)),uv_d(6,1:length(uv)),'x','Color',[46,188,89]/255,'linewidth',1); hold on
plot(uv_d(7,1:length(uv)),uv_d(8,1:length(uv)),'x','Color',[56,10,217]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$P_1$','$P_2$','$P_3$','$P_4$','$P_{1d}$','$P_{2d}$','$P_{3d}$','$P_{4d}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Evolucion Pixels}$','Interpreter','latex','FontSize',9);
ylabel('$v$','Interpreter','latex','FontSize',9);
xlabel('$u$','Interpreter','latex','FontSize',9);
xlim([-uo uo])
ylim([-vo vo])
set(gca,'Ydir','reverse')
print -dpng pixel
print -depsc pixel

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(he)),he,'linewidth',1); hold on;
grid('minor')
grid on;
legend({'$\tilde{P}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Evolution of Control Errors}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);
print -dpng error
print -depsc error

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(ul)),ul,'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(ul)),um,'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t(1:length(ul)),un,'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t(1:length(ul)),w,'Color',[83,57,217]/255,'linewidth',1); hold on

plot(t(1:length(hp)),hp(1,:),'--','Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(hp)),hp(2,:),'--','Color',[46,188,89]/255,'linewidth',1); hold on
plot(t(1:length(hp)),hp(3,:),'--','Color',[26,115,160]/255,'linewidth',1); hold on
plot(t(1:length(hp)),hp(6,:),'--','Color',[83,57,217]/255,'linewidth',1); hold on
grid('minor')
grid on;
% legend({'$\mu_{lc}$','$\mu_{mc}$','$\mu_{nc}$','$\omega_{c}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend({'$\mu_{lc}$','$\mu_{mc}$','$\mu_{nc}$','$\omega_{c}$','$\mu_{l}$','$\mu_{m}$','$\mu_{n}$','$\omega$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Control Values}$','Interpreter','latex','FontSize',9);
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);



figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(t_sample)),t_sample,'Color',[46,188,89]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$t_{sample}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Control Values}$','Interpreter','latex','FontSize',9);
ylabel('$[s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);  
print -dpng SAMPLE_TIME
print -depsc SAMPLE_TIME

% figure
% set(gcf, 'PaperUnits', 'inches');
% set(gcf, 'PaperSize', [4 2]);
% set(gcf, 'PaperPositionMode', 'manual');
% set(gcf, 'PaperPosition', [0 0 10 4]);
% plot(t(1:length(z_esti)),z_esti,'linewidth',1); hold on;
% grid('minor')
% grid on;
% legend({'${z_c}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
% legend('boxoff')
% title('$\textrm{Deep camera}$','Interpreter','latex','FontSize',9);
% ylabel('$[m]$','Interpreter','latex','FontSize',9);
% print -dpng Z_estimation
% print -depsc Z_estimation
% 
% figure
% set(gcf, 'PaperUnits', 'inches');
% set(gcf, 'PaperSize', [4 2]);
% set(gcf, 'PaperPositionMode', 'manual');
% set(gcf, 'PaperPosition', [0 0 10 4]);
% plot(t(1:length(z)),z,'linewidth',1); hold on;
% grid('minor')
% grid on;
% legend({'${z_c}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
% legend('boxoff')
% title('$\textrm{Deep camera}$','Interpreter','latex','FontSize',9);
% ylabel('$[m]$','Interpreter','latex','FontSize',9);
% print -dpng Z
% print -depsc Z
