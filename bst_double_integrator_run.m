close all
% clear all

%% Double Integrador - Model
[Ac, Bc, Cc] = tf2ss(1,[1 0 0]);
Gs = tf(1,[1 0 0]);
sysC = ss(Gs);
Ts = 1;
sysD = c2d(sysC,Ts,'zoh');
Ad = sysD.A; Bd = sysD.B; Cd = sysD.C;

Nstate = size(Ad,1);
Ncontrol = size(Bd,2);
Nout = size(Cd,1);
Nref = Nout;
Umax = 0.5;
Umin = -0.5;

N = 4;
% file_bst = ['bst_original_double_int_mpt3_', num2str(N), '_v2.mat'];
% load(file_bst)
% file_regions = ['regions_double_int_mpt3_', num2str(N), '_v2.mat'];
% load(file_regions)

%% Simulation Parameters
Nsim = 21;
x = zeros(Nstate,Nsim);
y = zeros(Nout,Nsim);
u = y;
deltau = y;

vet_index = zeros(1,Nsim);
erro = zeros(Nout,Nsim);
t = (0:(Nsim-1))*Ts;
cont_out = 0;
cont_reg = 0;
vetor_repeticao = zeros(1,Nsim);
ref = zeros(4,Nsim);

x(:,1) = [1.5 1]';


%% Simulation
for i = 1:Nsim
    i;
    index = 0;
    y(:,i) = Cd*x(:,i); 
    
    cont_reg(i) = 0;
    num_operation = 0;  
    
    vet_index(1,i) = index;
    vet_index(2,i) = num_operation;
    
    [vet_index_bst(1,i) vet_index_bst(2,i)] = evaluate_region_BST(x(:,i), nodes);
    index = vet_index_bst(1,i);
    
    if index ~= 0
        old_index = index;
    else
        index = old_index;
    end
    
   	u_calc = Regions{index,3}*x(:,i) + Regions{index,4};
    
    u(:,i) = u_calc(1);   
    x(:,i+1) = Ad*x(:,i) + Bd*u(:,i);
    
    if i==13
        x(:,i+1) = [1 -0.5]';
    end
end
    
%% Plot
%%%%%Plot Estados - X
figure
plot(t,x(1,1:length(t)))
hold on
plot(t,x(2,1:length(t)), '-r')
grid on
title('Estados - X1 e X2')

%%%%%Plot Saidas - Y
figure
plot(t,y)
grid on
title('Saida - Y')

%%%%%Plot A��es de Controle - U
figure
plot(t,u(1,:),'-b')
grid on
title('U')


%%
outputType = 'pdf';
%sim('Ex3');
set(groot,'DefaultTextInterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');
set(groot,'defaultAxesFontName', 'Times New Roman');
set(groot,'defaultAxesFontSize', 8);

%% State 
fig = figure();
set(fig,'Visible','off') 

plot(t,x(1,1:length(t)),'Color',[0,0,0],'LineWidth', 1.5)
hold on;
plot(t,x(2,1:length(t)),'--','Color',[0.35,0.35,0.35],'LineWidth', 1)
hold on;

h = legend('\bf{$x_1$}','\bf{$x_2$}','Interpreter','latex');
set(h,'Interpreter','latex','FontSize',50)
% set(h,'Position', [0.815 0.60 0.1 0.009])
set(h,'Location','northeast')

axis([0 (Nsim-1) -2 4])
set(gca,'FontSize', 40)
xlabel('Time [s]','FontSize',40)
set(gca,'Ytick',-2:2:4)
set(gca,'Xtick',0:5:(Nsim-1))
grid on

set(gca,'LooseInset',get(gca,'TightInset'));
set(fig, 'PaperUnits', 'centimeters')
set(fig, 'PaperSize', [17.5 13]);
set(fig, 'PaperPosition', [0 0 17.5 13]);
saveas(fig, 'D:\Documentos\OneDrive - UDESC Universidade do Estado de Santa Catarina\TCC\Artigo BST\figuras\resultados\result_state_bst_original_v2', outputType);

%% Control
fig = figure();
set(fig,'Visible','off') 

plot(t,u(1,1:length(t)),'Color',[0,0,0],'LineWidth', 1.5)
hold on;
plot(t,Umax*ones(1,length(t)),'--','Color',[0 0 0])
plot(t,Umin*ones(1,length(t)),'--','Color',[0 0 0])


h = legend('\bf{$u$}');
set(h,'Interpreter','latex','FontSize',50)
% set(h,'Position', [0.815 0.60 0.1 0.009])
set(h,'Location','northeast')

axis([0 (Nsim-1) -0.6 0.6])
set(gca,'FontSize', 40)
xlabel('Time [s]','FontSize',40)
set(gca,'Ytick',-0.5:0.5:0.5)
set(gca,'Xtick',0:5:(Nsim-1))
grid on

set(gca,'LooseInset',get(gca,'TightInset'));
set(fig, 'PaperUnits', 'centimeters')
set(fig, 'PaperSize', [17.5 13]);
set(fig, 'PaperPosition', [0 0 17.5 13]);
saveas(fig, 'D:\Documentos\OneDrive - UDESC Universidade do Estado de Santa Catarina\TCC\Artigo BST\figuras\resultados\result_control_bst_original_v2', outputType);
