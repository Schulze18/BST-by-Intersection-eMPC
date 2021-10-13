% clear all
close all
% clc

%% Model
% s = tf('s');
% Gs = 1/s^4;
% ts = 1;
% [A,B,C,D] = tf2ss(1,[1 0 0 0 0]);
% Gd = c2d(Gs,ts);

Ad = [4 -1.5 0.5 -0.25;
     4 0 0 0;
     0 2 0 0;
     0 0 0.5 0];
Bd = [0.5 0 0 0]';
Cd = [0.083 0.22 0.11 0.02];

%% Cost Function Weight
Q = eye(4);%diag([5 10 10 10]);
R = 0.01;
P = zeros(4);

%%
Nstate = size(Ad,1);
Ncontrol = size(Bd,2);
Nout = size(Cd,1);
Nref = 0;%Nout;

%% Constraints
Umax = 1;
Umin = -1;
Ymax = 10;
Ymin = -10;

%% Horizons
% Ny = 2;
% Nu = 2;
% init_ref = 1;

%% Model and Constraints in MPT3 formulation
model = LTISystem('A', Ad, 'B', Bd, 'C', Cd);
model.u.max = Umax;
model.u.min = Umin;
model.u.penalty = QuadFunction(R);
% model.y.penalty = QuadFunction(1);
% model.y.with('reference');
% model.y.reference = 'free'; 
model.y.max = Ymax;
model.y.min = Ymin;
model.x.penalty = QuadFunction(Q);
model.x.with('terminalPenalty');
model.x.terminalPenalty = QuadFunction(P);
N = 3;

%% MPT3
mpc = MPCController(model, N);
expmpc = mpc.toExplicit();

%% Save Region Data
Regions = {};
optimizer = expmpc.optimizer;
for i = 1:optimizer.Num
        Regions{i,1} = optimizer.Set(i).A;
        Regions{i,2} = optimizer.Set(i).b;
        Regions{i,3} = optimizer.Set(i).Functions('primal').F;
        Regions{i,4} = optimizer.Set(i).Functions('primal').g;
end

% name_file = strcat('regions_example5_mpt3_', num2str(N));
name_file = ['regions_example5_mpt3_', num2str(N), '.mat'];
save(name_file,'Regions')

tree = BinTreePolyUnion(optimizer);