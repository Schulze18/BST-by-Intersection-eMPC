clear all
% close all
% clc

% State Space Model
Ad = [0.7326 -0.0861;
      0.1722 0.9909];
Bd = [0.0609 0.0064]';
Cd = [0 1.4142];


%% Cost Function Weight
Q = eye(2);%diag([5 10 10 10]);
R = 0.01;

%%
Nstate = size(Ad,1);
Ncontrol = size(Bd,2);
Nout = size(Cd,1);
Nref = 0;%Nout;

%% Constraints
Umax = 1.5;
Umin = -1.5;
Xmax = [];
Xmin = [];

%% Horizons
Ny = 5;
Nu = 5;

%% Model and Constraints in MPT3 formulation
model = LTISystem('A', Ad, 'B', Bd, 'C', Cd);
model.u.max = Umax;
model.u.min = Umin;
model.x.penalty = QuadFunction(Q);
model.u.penalty = QuadFunction(R);
model.x.with('terminalPenalty');
model.x.terminalPenalty = QuadFunction(Q);
% model.y.penalty = QuadFunction(1);
% model.y.with('reference');
% model.y.reference = 'free'; 
% model.y.max = Ymax;
% model.y.min = Ymin;
N = Ny;

%% MPT3
mpc = MPCController(model, N);
tic
expmpc = mpc.toExplicit();
toc
%% Plot
figure
opt = expmpc.optimizer;
opt.plot()
xlim([-5 5])
ylim([-5 5])

%% BST
tree = BinTreePolyUnion(expmpc.optimizer);