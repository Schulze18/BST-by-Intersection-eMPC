clear all
close all
% clc

%K-d tree based approach for point location problem in explicit model predictive control

%% Model
Ad = [0.7 -0.1 0
      0.2 -0.5 0.1;
        0  0.1 0.1];
Bd = [0.1 0;
      0.1 0.1;
      0.1 0];
Cd = [1 0 0];
    
%%
Nstate = size(Ad,1);
Ncontrol = size(Bd,2);
Nout = size(Cd,1);
Nref = Nout;
    
%% Cost Function Weight
Q = eye(Nstate);
R = 0.01*eye(Ncontrol);
P = Q;

%% Constraints
Umax = [5 5];
Umin = [-5 -5];
Xmax = 100*ones(Nstate,1);
Xmin = -100*ones(Nstate,1);

%% Model and Constraints in MPT3 formulation
model = LTISystem('A', Ad, 'B', Bd, 'C', Cd);
model.u.max = Umax;
model.u.min = Umin;
model.u.penalty = QuadFunction(R);

model.x.max = Xmax;
model.x.min = Xmin;
model.x.penalty = QuadFunction(Q);
N = 5;

%% Explicit MPC
mpc = MPCController(model, N);
expmpc = mpc.toExplicit();

%% Save Region Data
Regions = {};
optimizer = expmpc.optimizer;
for i = 1:optimizer.Num
        Regions{i,1} = optimizer.Set(i).A;
        Regions{i,2} = optimizer.Set(i).b;
        Regions{i,3} = optimizer.Set(i).Functions('primal').F(Ncontrol,:);
        Regions{i,4} = optimizer.Set(i).Functions('primal').g(Ncontrol,:);
end

%%
name_file = ['regions_example2_kd_tree_mpt3_', num2str(N), '.mat'];
save(name_file,'Regions')

%% Verify Total Inequations and Control
tol = 1e-5;
ineq = verifiy_total_ineq(Regions);
fprintf('Number Inequalitis: %d\n', size(ineq,1));

list_control = list_control_laws(Regions, ineq, tol);
fprintf('Number Controls: %d\n', size(list_control,1));