% close all
clear all
% clc


%% System Model
% Base Area
A1 = 28;
A2 = 32;
A3 = 28;
A4 = 32;

% Output area
a1 = 0.071;
a3 = a1;
a2 = 0.057;
a4 = a2;

% gravity
g = 981;

% Gauge Constant
kc = 0.50;

% Output matrix 
Cc = [kc 0 0 0;
      0 kc 0 0];

% Pump Constant
k1 = 3.33;
k2 = 3.35;

% Flow partition
gamma1 = 0.70;
gamma2 = 0.60;

% h1_bar = 12.4;
% h2_bar = 12.7;
h1barra = 10.0;
h2barra = 10.0;

% v1barra = gama2*(sqrt(h1barra*a1^2*2*g)-(1-gama2)*sqrt(h2barra*a2^2*2*g))/(k1*(gama1+gama2-1));
v1barra = (sqrt(h1barra*a1^2*2*g)-(1-gamma2)/gamma2*sqrt(h2barra*a2^2*2*g))/(gamma1*k1-(1-gamma2)/gamma2*(1-gamma1)*k1);
v2barra = (sqrt(h2barra*a2^2*2*g)-(1-gamma1)*k1*v1barra)/(gamma2*k2);

h3barra = (1-gamma2)^2*k2^2*v2barra^2/(a3^2*2*g);
h4barra = (1-gamma1)^2*k1^2*v1barra^2/(a4^2*2*g);
%%
T1 = A1/a1*sqrt(2*h1barra/g);
T2 = A2/a2*sqrt(2*h2barra/g);
T3 = A3/a3*sqrt(2*h3barra/g);
T4 = A4/a4*sqrt(2*h4barra/g);

Ac = [-1/T1 0 A3/(A1*T3) 0;
       0  -1/T2 0 A4/(A2*T4);
       0  0 -1/T3 0;
       0  0 0 -1/T4];
Bc = [gamma1*k1/A1 0;
     0 gamma2*k2/A2;
     0 (1-gamma2)*k2/A3;
     (1-gamma1)*k1/A4 0];

% System Discretization
% Ts = 0.01;
Ts = 1;
sys = ss(Ac,Bc,Cc,0);
sysD = c2d(sys,Ts,'zoh');
Ad = sysD.A; Bd = sysD.B; Cd = sysD.C;

Nstate = size(Ad,1);
Ncontrol = size(Bd,2);
Nout = size(Cd,1);


%% Cost Function Weight
Q = eye(4);
R = 0.001*eye(2);
[trash,P] = dlqr(Ad,Bd,Q,R);

%% Constraints
Umax = [3 3]';
Umin = [-3 -3]';
% Xmax = [Inf Inf 0.44 0.6 Inf Inf]';
% Xmin = -[Inf Inf 0.44 0.6 Inf Inf]';


%% Horizons
Ny = 5;
Nu = 2;

%%
% options = mptopt;
% options.report_period = 120;
% options.region_tol = 1e-6;
% options.zero_tol = 1e-10;
% options.abs_tol = 1e-7;

%% Model and Constraints in MPT3 formulation
model = LTISystem('A', Ad, 'B', Bd, 'C', Cd);
model.u.max = Umax;
model.u.min = Umin;
model.u.penalty = QuadFunction(R);
% model.x.max = Xmax;
% model.x.min = Xmin;
model.x.penalty = QuadFunction(Q);
% model.x.with('terminalPenalty');
% model.x.terminalPenalty = QuadFunction(P);
model.u.with( 'block' );
model.u.block.from = Nu;
model.u.block.to = Ny;
%% Explicit MPC
mpc = MPCController(model, Ny);
expmpc = mpc.toExplicit();

%% Save Region Data
Regions = {};
optimizer = expmpc.optimizer;
total_ineq = 0;
for i = 1:optimizer.Num
        Regions{i,1} = optimizer.Set(i).A;
        Regions{i,2} = optimizer.Set(i).b;
        Regions{i,3} = optimizer.Set(i).Functions('primal').F(1:Ncontrol,:);
        Regions{i,4} = optimizer.Set(i).Functions('primal').g(1:Ncontrol,:);
        total_ineq = total_ineq + size(optimizer.Set(i).A,1);
end

%%
% % ineq_raw_list = [];
% % for i = 1:optimizer.Num
% %     num_ineq = size(optimizer.Set(i).A,1);
% %     ineq_raw_list = [ineq_raw_list; optimizer.Set(i).A optimizer.Set(i).b ones(num_ineq,1)*i];
% % %     total_ineq = total_ineq + size(optimizer.Set(i).A,1);
% % end

%%
% tic
% tree = BinTreePolyUnion(optimizer);
% toc
% % % 
%% Verify Total Inequations and Control
tol = 1e-6;
ineq = verifiy_total_ineq(Regions);
fprintf('Number Inequalitis: %d\n', size(ineq,1));

% ineq_mpt3 = mpt3_verify_total_ineq_v2(optimizer,7);
% fprintf('Number Inequalitis with MPT3 algorithm: %d\n', size(ineq_mpt3,1));


%
list_control = list_control_laws(Regions, ineq, tol);
fprintf('Number Controls: %d\n', size(list_control,1));


%% Simulate
% X = []; U = []; Y = [];
% x0 = [-1.5 1.1 -0.2 0.1]';
% model.initialize(x0)
% Tsim = 200;
% Nsim = Tsim/Ts;
% for i=1:Nsim
%    u = mpc.evaluate(x0);
%    [x0, y] = model.update(u);
%    X = [X; x0'];
%    U = [U; u'];
%    Y = [Y; y'];
% end
% % Plot
% time1 = 0:Ts:Tsim;
% figure
% plot(time1(1:size(X(:,1),1)),X(:,1))
% hold on
% plot(time1(1:size(X(:,1),1)),X(:,2))
% plot(time1(1:size(X(:,1),1)),X(:,3))
% plot(time1(1:size(X(:,1),1)),X(:,4))
% title('Estados MPC')
% grid on
% %%
% figure
% plot(time1(1:size(X(:,1),1)),U(:,1))
% hold on
% plot(time1(1:size(X(:,1),1)),U(:,2))
% title('Control MPC')
% grid on

%%
name_file = ['regions_quadruple_tank_mpt3_', '_Ny_', num2str(Ny), '_Nu_',  num2str(Nu), '.mat'];
save(name_file,'Regions')