clear all
close all
% clc

% % % %% Fourth Order Integrador - Model
% % % [Ac, Bc, Cc] = tf2ss(1,[1 0 0 0 0]);
% % % Gs = tf(1,[1 0 0 0 0]);
% % % sysC = ss(Gs);
% % % Ts = 1;
% % % sysD = c2d(sysC,Ts,'zoh');
% % % Ad = sysD.A; Bd = sysD.B; Cd = sysD.C;

Ad = [4 -1.5 0.5 -0.25; 
      4 0 0 0;
      0 2 0 0;
      0 0 0.5 0];
Bd = [0.5 0 0 0]';
Cd = [0.083 0.22 0.11 0.02];
%%
Nstate = size(Ad,1);
Ncontrol = size(Bd,2);
Nout = size(Cd,1);
Nref = Nout;



%% Cost Function Weight
Q = eye(Nstate);%diag([5 10 10 10]);
R = 0.01;
P = zeros(Nstate);%eye(2);

%% Constraints
Umax = 1;
Umin = -1;
Ymax = 10;
Ymin = -10;

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
model.x.max = Ymax*ones(4,1);
model.x.min = Ymin*ones(4,1);
model.x.penalty = QuadFunction(Q);
% model.x.with('terminalPenalty');
% model.x.terminalPenalty = QuadFunction(P);
N = 6;
Ny = N; Nu = N;
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

% name_file = strcat('regions_example5_mpt3_', num2str(N));
% name_file = ['regions_double_int_mpt3_', num2str(N), '_v3.mat'];
% save(name_file,'Regions')
% % % 
% % % tree = BinTreePolyUnion(optimizer);
% % % 
%% Verify Total Inequations and Control
tol = 1e-6;
ineq = verifiy_total_ineq(Regions);
fprintf('Number Inequalitis: %d\n', size(ineq,1));

list_control = list_control_laws(Regions, ineq, tol);
fprintf('Number Controls: %d\n', size(list_control,1));

%%
% name_file = ['regions_quad_int_mpt3_', '_Ny_', num2str(Ny), '_Nu_',  num2str(Nu), '.mat'];
% save(name_file,'Regions')
