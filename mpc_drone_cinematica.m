function [f,solver,args] = mpc_drone_cinematica(bounded, N, L1, param, chi, ts)

addpath('/home/fer/casadi-linux-matlabR2014b-v3.4.5');
import casadi.*;

%% Dynamic Parameters
%% Dinamic Parameters
x1 = chi(1);
x2 = chi(2);
x3 = chi(3);
x4 = chi(4);
x5 = chi(5);
x6 = chi(6);
x7 = chi(7);
x8 = chi(8);
x9 = chi(9);
x10 = chi(10);
x11 = chi(11);
x12 = chi(12);
x13 = chi(13);
x14 = chi(14);
x15 = chi(15);
x16 = chi(16);
x17 = chi(17);
x18 = chi(18);
x19 = chi(19);
x20 = chi(20);
x21 = chi(21);
x22 = chi(22);
x23 = chi(23);
x24 = chi(24);
x25 = chi(25);
x26 = chi(26);
x27 = chi(27);

%% Definicion de las restricciones en las acciones de control
%% RESTRICCION ROBOT 1
ul1_max = bounded(1); 
ul1_min = bounded(2);

um1_max = bounded(3);
um1_min = bounded(4);

un1_max = bounded(5);
un1_min = bounded(6);

w1_max = bounded(7); 
w1_min = bounded(8);



%% Generacion de las variables simbolicas de los estados del sistema
u1 = MX.sym('u1');
v1 = MX.sym('v1');

u2 = MX.sym('u2');
v2 = MX.sym('v2');

u3 = MX.sym('u3');
v3 = MX.sym('v3');

u4 = MX.sym('u4');
v4 = MX.sym('v4');


x1c = MX.sym('x1c'); 
y1c = MX.sym('y1c');
z1c = MX.sym('z1c');

x2c = MX.sym('x2c'); 
y2c = MX.sym('y2c');
z2c = MX.sym('z2c');

x3c = MX.sym('x3c'); 
y3c = MX.sym('y3c');
z3c = MX.sym('z3c');

x4c = MX.sym('x4c'); 
y4c = MX.sym('y4c');
z4c = MX.sym('z4c');



th = MX.sym('th');

ul = MX.sym('ul');
um = MX.sym('um');
un = MX.sym('un');
w = MX.sym('w');

%% Aux vector help the contruction of the jacobians matrices
U_m = [u1; v1; u2; v2; u3; v3; u4; v4];
H = [z1c; z2c; z3c; z4c];
obj_c = [x1c; y1c; z1c; x2c; y2c; z2c; x3c; y3c; z3c; x4c; y4c; z4c];

%% Definicion de cuantos estados en el sistema
states = [u1;v1;u2;v2;u3;v3;u4;v4;x1c;y1c;z1c;x2c;y2c;z2c;x3c;y3c;z3c;x4c;y4c;z4c;th;ul;um;un;w];
n_states = length(states);

%% Generacion de las variables simbolicas de las acciones del control del sistema
ul1_ref = MX.sym('ul1_ref');
um1_ref = MX.sym('um1_ref');
un1_ref = MX.sym('un1_ref');
w1_ref = MX.sym('w1_ref');

%% Defincion de cuantas acciones del control tiene el sistema
controls = [ul1_ref;um1_ref;un1_ref;w1_ref]; 
n_control = length(controls);


%% Definicion de los las constantes dl sistema
a = L1(1);
b = L1(2);

fx = param(1);
fy = param(2);
u0 = param(3);
v0 = param(4);

%% Defincion del sistema pero usando espacios de estados todo el sistema de ecuaciones
aux=1;
J_p = [];
for k=1:2:length(U_m)
    ii=aux;
    i = k;
    j = k+1;
    
    J_manual_p = [-fx/(H(ii)), 0, U_m(i)/H(ii), U_m(i)*U_m(j)/fx, -(fx^2+U_m(i)^2)/(fx), U_m(j), 0;...
        0, -(fy)/(H(ii)), U_m(j)/H(ii), (fy^2+U_m(j)^2)/(fy), -(U_m(i)*U_m(j))/(fy), -U_m(i), 0];
    
    
    J_p =[J_p;J_manual_p];
    
    aux=aux+1;
end

J_c = [];
for k=1:3:length(obj_c)
    x = k;
    y = k + 1;
    z = k + 2;
    J_manual_c = [-1, 0, 0, 0, -obj_c(z), obj_c(y),0;...
        0, -1, 0, obj_c(z), 0, -obj_c(x),0;...
        0, 0, -1, -obj_c(y), obj_c(x), 0,0];
    
    J_c =[J_c;J_manual_c];
end

J_th = [0,0,0,0,0,0,1];

J = [J_p;J_c;J_th];

R_1 = [sin(th), -cos(th), 0;...
    -cos(th), -sin(th), 0;...
    0, 0, -1];

R_total = [R_1, zeros(3,3);...
    zeros(3,3), R_1;...
    zeros(1,6)];

R_total(7,6) = 1;

T = [1,0,0,0;...
    0,1,0,0;...
    0,0,1,0;...
    0,0,0,0;...
    0,0,0,0;...
    0,0,0,1];

J_drone = [cos(th), -sin(th), 0, -(a*sin(th)+b*cos(th));...
     sin(th), cos(th), 0,  (a*cos(th)-b*sin(th));...
     0, 0, 1, 0;...
     0, 0, 0, 1]; 
 
J_camara = J*R_total*T*J_drone;

M_1 = [x6/(x1*x6 - x2*x5), 0, 0, -x2/(x1*x6 - x2*x5);...
       0, 1/x3, 0, 0;...
       0, 0, 1/x4, 0;...
      -x5/(x1*x6 - x2*x5), 0, 0, x1/(x1*x6 - x2*x5)];

%% CENTRIFUGAL FORCES
C = [x7, x8 + w*x9, x10, x11;...
    x12 + w*x13, x14, x15, x16 + w*x17;...
    x18, x19, x20, x21;...
    x22, x23 + w*x24, x25, x26];

G = [0; 0; x27; 0];

% A = [zeros(21,21),J_camara;...
%      zeros(4,21),-M_1*C];
%  
% B = [zeros(21,4);...
%     M_1];
% 
% aux_m = [zeros(21,1);...
%        -M_1*G];

At = [-0.2410    0.0178    0.0021    0.0037;...
      -0.0022   -0.3364   -0.0178    0.0275;...
      0.0023   -0.0096   -0.2960    0.0041;...
      -0.0002    0.1930    0.0504   -0.6240];
  
Bt = [0.4528   -0.0161   -0.0097   -0.0195;...
      0.0040    0.4916    0.0128   -0.0370;...
     -0.0033    0.0265    0.3307   -0.0092;...
      0.0078   -0.3065   -0.0417    0.5536];
A = [zeros(21,21),J_camara;...
     zeros(4,21),At];
 
B = [zeros(21,4);...
    Bt];


% rhs=(A*states+aux_m+B*controls);

rhs=(A*states+B*controls);


%% Definicion de kas funciones del sistema
f = Function('f',{states,controls},{rhs}); 
U = SX.sym('U',n_control,N);
P = SX.sym('P',n_states + N*(n_states));
%% vector que incluye el vector de estados y la referencia
X = SX.sym('X',n_states,(N+1));

%% Vector que representa el problema de optimizacion
g = [];  % restricciones de estados del problema  de optimizacion

%%EMPY VECTOR ERRORS
he = [];

%% EMPY VECTOR CONTROL VALUES
u = [];

%% INITIAL CONDITION ERROR
st  = X(:,1); % initial state
g = [g;X(:,1)-P(1:25)]; % initial condition constraints

%% Definicon del bucle para optimizacion a los largo del tiempo
for k = 1:N
    st = X(:,k);  con = U(:,k);
    
    hd = P(25*k+1:25*k+8);
    
    h = X(1:8,k);

    %% Funcion costo a minimizar 
    he = [he;h-hd];
    u = [u;con];
    
    %% Actualizacion del sistema usando Euler runge kutta
    st_next = X(:,k+1);
    k1 = f(st, con);   % new 
    k2 = f(st + ts/2*k1, con); % new
    k3 = f(st + ts/2*k2, con); % new
    k4 = f(st + ts*k3, con); % new
    st_next_RK4=st +ts/6*(k1 +2*k2 +2*k3 +k4); % new 
    
    %% Restricciones del sistema se =basan en el modelo del sistema
    g = [g;st_next-st_next_RK4]; 
end

%% Cost final 
Q = (1/fx)*eye(size(he,1));
R = 50*eye(size(u,1));

%% FINAL COST
obj = he'*Q*he+u'*R*u;


% se crea el vector de desiscion solo de una columna
OPT_variables = [reshape(X,25*(N+1),1);reshape(U,4*N,1)];

nlprob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlprob,opts);

args = struct;

args.lbg(1:25*(N+1)) = 0;  %-1e-20  %Equality constraints
args.ubg(1:25*(N+1)) = 0;  %1e-20   %Equality constraints


%% DRONE 1
args.lbx(1:25:25*(N+1),1) = -u0; %state u1 lower bound
args.ubx(1:25:25*(N+1),1) = u0;  %state u1 upper bound

args.lbx(2:25:25*(N+1),1) = -v0; %state v1 lower bound
args.ubx(2:25:25*(N+1),1) = v0;  %state v1 upper bound

args.lbx(3:25:25*(N+1),1) = -u0; %state u2 lower bound
args.ubx(3:25:25*(N+1),1) = u0;  %state u2 upper bound

args.lbx(4:25:25*(N+1),1) = -v0; %state v2 lower bound
args.ubx(4:25:25*(N+1),1) = v0;  %state v2 lower bound

args.lbx(5:25:25*(N+1),1) = -u0; %state u3 lower bound
args.ubx(5:25:25*(N+1),1) = u0;  %state u3 lower bound

args.lbx(6:25:25*(N+1),1) = -v0; %state v3 lower bound
args.ubx(6:25:25*(N+1),1) = v0;  %state v3 lower bound

args.lbx(7:25:25*(N+1),1) = -u0; %state u4 lower bound
args.ubx(7:25:25*(N+1),1) = u0;  %state u4 upper bound

args.lbx(8:25:25*(N+1),1) = -v0; %state v4
args.ubx(8:25:25*(N+1),1) = v0;  %state v4

args.lbx(9:25:25*(N+1),1) = -inf; %state u5
args.ubx(9:25:25*(N+1),1) = inf;  %state u5

args.lbx(10:25:25*(N+1),1) = -inf; %state v5
args.ubx(10:25:25*(N+1),1) = inf;  %state v5

args.lbx(11:25:25*(N+1),1) = -inf; %state z3 lower bound
args.ubx(11:25:25*(N+1),1) = inf;  %state z3 upper bound

args.lbx(12:25:25*(N+1),1) = -inf; %state theta3 lower bound
args.ubx(12:25:25*(N+1),1) = inf;  %state theta3 upper bound

args.lbx(13:25:25*(N+1),1) = -inf; %state ul1 lower bound
args.ubx(13:25:25*(N+1),1) = inf;  %state ul1 upper bound

args.lbx(14:25:25*(N+1),1) = -inf; %state um1 lower bound
args.ubx(14:25:25*(N+1),1) = inf;  %state um1 upper bound

args.lbx(15:25:25*(N+1),1) = -inf; %state un1 lower bound
args.ubx(15:25:25*(N+1),1) = inf;  %state un1 upper bound

args.lbx(16:25:25*(N+1),1) = -inf; %state w1 lower bound
args.ubx(16:25:25*(N+1),1) = inf;  %state w1 upper bound

args.lbx(17:25:25*(N+1),1) = -inf; %state ul2 lower bound
args.ubx(17:25:25*(N+1),1) = inf;  %state ul2 upper bound

args.lbx(18:25:25*(N+1),1) = -inf; %state um2 lower bound
args.ubx(18:25:25*(N+1),1) = inf;  %state um2 upper bound

args.lbx(19:25:25*(N+1),1) = -inf; %state un2 lower bound
args.ubx(19:25:25*(N+1),1) = inf;  %state un2 upper bound

args.lbx(20:25:25*(N+1),1) = -inf; %state w2 lower bound
args.ubx(20:25:25*(N+1),1) = inf;  %state w2 upper bound

args.lbx(21:25:25*(N+1),1) = -inf; %state ul3 lower bound
args.ubx(21:25:25*(N+1),1) = inf;  %state ul3 upper bound

args.lbx(22:25:25*(N+1),1) = -inf; %state um3 lower bound
args.ubx(22:25:25*(N+1),1) = inf;  %state um3 upper bound

args.lbx(23:25:25*(N+1),1) = -inf; %state un3 lower bound
args.ubx(23:25:25*(N+1),1) = inf;  %state un3 upper bound

args.lbx(24:25:25*(N+1),1) = -inf; %state w3 lower bound
args.ubx(24:25:25*(N+1),1) = inf;  %state w3 upper bound

args.lbx(25:25:25*(N+1),1) = -inf; %state w3 lower bound
args.ubx(25:25:25*(N+1),1) = inf;  %state w3 upper bound



%% Definicion de las restricciones de las acciones de control del sistema
% DRONE 1
args.lbx(25*(N+1)+1:4:25*(N+1)+4*N,1) = ul1_min;  %
args.ubx(25*(N+1)+1:4:25*(N+1)+4*N,1) = ul1_max;  %

args.lbx(25*(N+1)+2:4:25*(N+1)+4*N,1) = um1_min;  %
args.ubx(25*(N+1)+2:4:25*(N+1)+4*N,1) = um1_max;  % 

args.lbx(25*(N+1)+3:4:25*(N+1)+4*N,1) = un1_min;  %
args.ubx(25*(N+1)+3:4:25*(N+1)+4*N,1) = un1_max;  %

args.lbx(25*(N+1)+4:4:25*(N+1)+4*N,1) = w1_min;  %
args.ubx(25*(N+1)+4:4:25*(N+1)+4*N,1) = w1_max;  %

end