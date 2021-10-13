clear all
close all
% clc

%% Double Integrador - Model
[Ac, Bc, Cc] = tf2ss(1,[1 0 0]);
Gs = tf(1,[1 0 0]);
sysC = ss(Gs);
Ts = 1;
sysD = c2d(sysC,Ts,'zoh');
Ad = sysD.A; Bd = sysD.B; Cd = sysD.C;

%%
Nstate = size(Ad,1);
Ncontrol = size(Bd,2);
Nout = size(Cd,1);
Nref = Nout;

%% Cost Function Weight
Q = eye(2);
R = 0.01;
P = Q;

%% Constraints
Umax = 0.5;
Umin = -0.5;

%% Model and Constraints in MPT3 formulation
model = LTISystem('A', Ad, 'B', Bd, 'C', Cd);
model.u.max = Umax;
model.u.min = Umin;
model.u.penalty = QuadFunction(R);
model.x.penalty = QuadFunction(Q);
model.x.with('terminalPenalty');
model.x.terminalPenalty = QuadFunction(P);
N = 4;

%% Explicit MPC
mpc = MPCController(model, N);
expmpc = mpc.toExplicit();

%% Save Region Data
Regions = {};
optimizer = expmpc.optimizer;

for i = 1:optimizer.Num
        Regions{i,1} = optimizer.Set(i).A;
        Regions{i,2} = optimizer.Set(i).b;
        Regions{i,3} = optimizer.Set(i).Functions('primal').F(1:Ncontrol,:);
        Regions{i,4} = optimizer.Set(i).Functions('primal').g(1:Ncontrol,:);
end

%%
name_file = ['regions_double_int_mpt3_', num2str(N), '.mat'];
save(name_file,'Regions')
% % % 
% % % tree = BinTreePolyUnion(optimizer);
% % % 
%% Verify Total Inequations and Control
tol = 1e-5;
ineq = verifiy_total_ineq(Regions);
fprintf('Number Inequalitis: %d\n', size(ineq,1));

list_control = list_control_laws(Regions, ineq, tol);
fprintf('Number Controls: %d\n', size(list_control,1));

