close all
clear all
% clc

Ad = [1 0 0.01 0 0 0;
      0 1 0 0.01 0 0;
      0 0 1 0 0 0;
      0 0 0 1 0 0;
      0.01 0 0 0 1 0;
      0 0.01 0 0 0 1];
Bd = [0 0;
        0.0001 -0.0001;
        0.0019 0.0019;
        0.0132 -0.0132;
        0 0;
        0 0];
Cd = [1 0 0 0 0 0;
      0 1 0 0 0 0];
Nstate = size(Ad,1);
Ncontrol = size(Bd,2);


Ts = 0.01;
Ac = (Ad - eye(Nstate))/Ts;  
%% Cost Function Weight
Q = 50*diag([100 100 10 10 400 200]);
R = eye(2);
% [trash, P ] = lqrd(Ac,Bd/Ts,Q,R,Ts); 
[trash,P] = dlqr(Ad,Bd,Q,R);


%% Constraints
Umax = [3 3]';
% Umin = [-1 -1]';
Umin = [-3 -3]';
% Xmax = [Inf Inf 0.44 0.6 Inf Inf]';
% Xmin = -[Inf Inf 0.44 0.6 Inf Inf]';
Xmax = [2*pi 2*pi 0.44 0.6 10 10]';
Xmin = -[2*pi 2*pi 0.44 0.6 10 10]';

%% Horizons
Ny = 5;
Nu = 2;

%% Model and Constraints in MPT3 formulation
model = LTISystem('A', Ad, 'B', Bd, 'C', Cd);
model.u.max = Umax;
model.u.min = Umin;
model.u.penalty = QuadFunction(R);
% model.x.max = 1000*ones(6,1);%Xmax;
% model.x.min = -1000*ones(6,1);%Xmin;
model.x.penalty = QuadFunction(Q);
model.x.with('terminalPenalty');
model.x.terminalPenalty = QuadFunction(P);
model.u.with( 'block' );
model.u.block.from = Nu;
model.u.block.to = Ny;
%% Explicit MPC
mpc = MPCController(model, Ny);
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

% name_file = strcat('regions_example5_mpt3_', num2str(N));
% name_file = ['regions_double_int_mpt3_', num2str(N), '_v3.mat'];
% save(name_file,'Regions')
% % % 
% % % tree = BinTreePolyUnion(optimizer);
% % % 
%% Verify Total Inequations and Control
tol = 1e-5;
ineq = verifiy_total_ineq(Regions);
fprintf('Number Inequalitis: %d\n', size(ineq,1));
% 
list_control = list_control_laws(Regions, ineq, tol);
fprintf('Number Controls: %d\n', size(list_control,1));

% %%
% Nsim = 1000;
% x0 = [0.5 0.5 0.0 0.0 0.0 0.0]';
% Y = []; U = []; X = [];
% model.initialize(x0);
% for i=1:Nsim
% 
% %    u = expmpc.evaluate(x0);
%    u = mpc.evaluate(x0);
%    [x0, y] = model.update(u);
%    X = [X; x0'];
%    U = [U; u'];
%    Y = [Y; y'];
% end
% 
% time = (0:(Nsim-1))*Ts;
% plot(time, X(:,1))
% hold on
% plot(time, X(:,2))
% plot(time, X(:,3))
% plot(time, X(:,4))
% plot(time, X(:,5))
% plot(time, X(:,6))
% title('Estados')
% grid on
% 
% figure
% plot(time, Y(:,1))
% hold on
% plot(time, Y(:,2))
% % ylim([-1 1])
% title('saida')
% grid on
% 
% figure
% plot(time, U(:,1))
% hold on
% plot(time, U(:,2))
% ylim([-1 3])
% title('control')
% grid on