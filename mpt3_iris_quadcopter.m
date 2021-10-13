% close all
clc
clear all

% Iris Quadcopter Model
m = 1.37;
Jxx = 0.0219;
Jyy = 0.0109;
Jzz = 0.0306;
Ts = 0.01;
Ac_quad = [0 1 0 0 0 0 0 0;
           0 0 0 0 0 0 0 0;
           0 0 0 1 0 0 0 0;
           0 0 0 0 0 0 0 0;
           0 0 0 0 0 1 0 0;
           0 0 0 0 0 0 0 0;
           0 0 0 0 0 0 0 1;
           0 0 0 0 0 0 0 0];
Bc_quad = [0 0 0 0;
     1/m 0 0 0;
     0 0 0 0;
     0 1/Jxx 0 0;
     0 0 0 0;
     0 0 1/Jyy 0;
     0 0 0 0;
     0 0 0 1/Jzz];
Cc_quad = [1 0 0 0 0 0 0 0;
           0 0 1 0 0 0 0 0;
           0 0 0 0 1 0 0 0;
           0 0 0 0 0 0 1 0];
Dc_quad = zeros(4,4);
sysC = ss(Ac_quad, Bc_quad, Cc_quad, Dc_quad);
sysD = c2d(sysC,Ts,'zoh');
Ad_quad = sysD.A; Bd_quad = sysD.B; Cd_quad = sysD.C;

Ad = [Ad_quad zeros(8,4);
    -Ts*Cd_quad eye(4)];
Bd = [Bd_quad; zeros(4,4)];
Cd = [Cd_quad zeros(4,4)];

Nstate = size(Ad,1);
Ncontrol = size(Bd,2);
Nout = size(Cd,1);
Nref = Nout;

%% Cost Function Weight
Q = blkdiag(zeros(8), diag([500, 0.1, 0.1, 0.1]));%diag([1 1 1 0.5]));
% Q = blkdiag(diag([0 0.01 0 0.01 0 0.01 0 0.01]), diag([5, 0.1, 0.1, 0.1]));
% Q = eye(12);%blkdiag(eye(8), zeros(4));
R = diag([0.0004,0.1,0.1,0.1]);%diag([0.001 0.1 0.1 0.1]);
P = Q;%eye(2);


%% Constraints
Umax = [30 Inf(1,3)];%[30 0.50 0.50 0.50];
Umin = [-15 -Inf(1,3)];%[-15 -0.50 -0.50 -0.50];
%Ymax = 3;
%Ymin = -3;
%Xmax
%Xmin


%% Model and Constraints in MPT3 formulation
N = 3;
model = LTISystem('A', Ad, 'B', Bd, 'C', Cd);
model.u.max = Umax;
model.u.min = Umin;
model.u.penalty = QuadFunction(R);
model.x.penalty = QuadFunction(Q);
% model.x.with('terminalPenalty');
% model.x.terminalPenalty = QuadFunction(P);

% % add a move-blocking constraint
% model.u.with('block');
% model.u.block.from = 3;
% model.u.block.to = N;

% model.x.with('reference');
% model.x.reference = 'free';

%% MPC
mpc = MPCController(model, N);

%% Optimization Problem Matrices
options = optimoptions('quadprog','Display','None');
PHI = Ad;
Hx = eye(Nstate);
Hu = zeros(Nstate,Ncontrol*(N));
Qbar = Q;
Rbar = R;
for i = 1:N-1
    Hx = [Hx; PHI^i];
    Qbar = blkdiag(Qbar, Q);
    Rbar = blkdiag(Rbar, R);
    
    Hu_temp = [];
    Hw_temp = [];
    for j = 1:N
        if i >= j
            Hu_temp = [Hu_temp PHI^(i-j)*Bd];
        else
            Hu_temp = [Hu_temp zeros(Nstate,Ncontrol)];
        end
    end
    Hu = [Hu; Hu_temp];
end

Hqp = Hu'*Qbar*Hu + Rbar;
Fqp = Hx'*Qbar*Hu;

%% Explicit MPC
% expmpc = mpc.toExplicit();

%%
Klqr = [2.2207    2.2248    0.0000    0.0000   -0.0000   -0.0000   -0.0000   -0.0000   -0.9919   -0.0000   -0.0000    0.0000;
    0.0000    0.0000    0.0556    0.0492    0.0000    0.0000   -0.0000   -0.0000   -0.0000   -0.0313   -0.0000    0.0000;
   -0.0000   -0.0000    0.0000    0.0000    0.0440    0.0309    0.0000    0.0000    0.0000   -0.0000   -0.0312   -0.0000;
   -0.0000   -0.0000   -0.0000   -0.0000    0.0000    0.0000    0.0622    0.0616    0.0000    0.0000   -0.0000   -0.0313];

K_lqr_new = dlqr(Ad,Bd,Q,R);
%% Simulate the closed loop
Nsim = 3000;

x0 = [0; zeros(7,1)];
ref = [1; 0; 0; 0];
x_quad = zeros(8,Nsim);
x_quad(:,1) = x0;
y_quad = zeros(4,Nsim);
y_quad(:,1) = Cd_quad*x0;
error = zeros(4,Nsim);
error(:,1) = ref - y_quad(:,1);
u = zeros(4,Nsim);
Uold = zeros(4,1);
for i = 1:Nsim
    y_quad(:,i) = Cd_quad*x_quad(:,i);
    if i>1
        error(:,i) = (ref-y_quad(:,i))*Ts + error(:,i-1);
%         error(:,i) = -y_quad(:,i)*Ts + error(:,i-1);
    end
    x(:,i) = [x_quad(:,i); error(:,i)];
%     u(:,i) = mpc.evaluate(x(:,i));
    
    %Quadprog
    U_array = quadprog(Hqp, x(:,i)'*Fqp, [], [], [], [], [], [], [], options);
    u(:,i) = U_array(1:Ncontrol,1);
%     Uold = u(:,i);
%     u(:,i) = -Klqr*x(:,i);
%     u(:,i) = -K_lqr_new*x(:,i);
    x_quad(:,i+1) = Ad_quad*x_quad(:,i) + Bd_quad*u(:,i);
%     x(:,i+1) = Ad*x(:,i) + Bd*u(:,i);
end

%% plot the output trajector%ies
figure
plot((0:Nsim-1)*Ts,y_quad,'LineWidth',1)
xlabel('Simulation steps')
title('Outputs')

%% plot the Control trajector%ies
figure
plot((0:Nsim-1)*Ts,u(1,:),'LineWidth',1)
xlabel('Simulation steps')
title('Control')