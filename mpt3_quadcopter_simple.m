clear all
close all
clc


%Iris 3DR Parameters
g = 9.81;
m = 1.37;
lx = 0.13;
ly = 0.21;
KT = 15.67e-6;
KD = 2.55e-7;
Im = 2.52e-10;
Ixx = 0.0219;
Iyy = 0.0109;
Izz = 0.0306;
Ax = 0.25;
Ay = 0.25;
Az = 0.25;
wo = 463; %Angular velocity that compensate the drones weigth
Ts = 0.01;

%State Space Linearized
Ac = [0 1 0 0 0 0 0 0;
     0 -Az/m 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0;
     0 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 1;
     0 0 0 0 0 0 0 0];
 Bc = [0 0 0 0;
      1/m 0 0 0;
      0 0 0 0;
      0 1/Ixx 0 0;
      0 0 0 0;
      0 0 1/Iyy 0;
      0 0 0 0;
      0 0 0 1/Izz];
Cc = [1 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0;
     0 0 0 0 1 0 0 0;
     0 0 0 0 0 0 1 0];

 %Discrete State Space
[Ad,Bd] = c2d(Ac,Bc,Ts);
Cd = Cc;
 
Nstate = size(Ad,1);
Ncontrol = size(Bd,2);
Nout = size(Cd,1);
Nref = Nout;


%% Extended Model
Ae = [Ad zeros(Nstate,Nout);
      -Ts*Cd eye(Nout)];
Be = [Bd; zeros(Nout,Ncontrol)];
Ce = [Cd zeros(Nout,Nout)];

%% Cost Function Weight
Q = blkdiag(zeros(Nstate),diag([1 0.1 0.1 0.1]));%diag([5 10 10 10]);
R = 0.01*eye(Ncontrol);
P = Q;%eye(2);
Umax = [30 0.5 0.5 0.3];

%% Model and Constraints in MPT3 formulation
N = 2;

model = LTISystem('A', Ae, 'B', Be, 'C', Ce);
ctrl = MPCController(model);
% set the prediction horizon
ctrl.N = N;

ctrl.model.u.max = Umax;
ctrl.model.u.min = -Inf(4,1);

ctrl.model.u.penalty = QuadFunction(R);
% % % % % % % % % % % % ctrl.model.u.with('block');
% % % % % % % % % % % % % ctrl.model.u.block.from = 2;
% % % % % % % % % % % % % ctrl.model.u.block.to = ctrl.N;

% model.y.penalty = QuadFunction(1);
% model.y.with('reference');
% model.y.reference = 'free'; 
ctrl.model.y.max = Inf(4,1);
ctrl.model.y.min = -Inf(4,1);
% model.x.max = [Ymax; Ymax];
% model.x.min = [Ymin; Ymin];
ctrl.model.x.penalty = QuadFunction(Q);
ctrl.model.x.with('terminalPenalty');
ctrl.model.x.terminalPenalty = QuadFunction(P);
% N = 3;

%% Explicit MPC
% mpc = MPCController(model, N);
expmpc = ctrl.toExplicit();

%% Save Region Data
Regions = {};
optimizer = expmpc.optimizer;
for i = 1:optimizer.Num
        Regions{i,1} = optimizer.Set(i).A;
        Regions{i,2} = optimizer.Set(i).b;
        Regions{i,3} = optimizer.Set(i).Functions('primal').F;
        Regions{i,4} = optimizer.Set(i).Functions('primal').g;
end