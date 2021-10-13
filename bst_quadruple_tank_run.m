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
Ts = 1;
sys = ss(Ac,Bc,Cc,0);
sysD = c2d(sys,Ts,'zoh');
Ad = sysD.A; Bd = sysD.B; Cd = sysD.C;

Nstate = size(Ad,1);
Ncontrol = size(Bd,2);
Nout = size(Cd,1);
%%
Ny = 8;
Nu = 2;
% file_bst = ['bst_intersec_quadruple_tank_mpt3_', '_Ny_', num2str(Ny), '_Nu_',  num2str(Nu), '.mat'];
file_bst = ['bst_original_quadruple_tank_mpt3_', '_Ny_', num2str(Ny), '_Nu_',  num2str(Nu), '.mat'];

load(file_bst)
% file_regions = ['regions_quadruple_tank_mpt3_', '_Ny_', num2str(Ny), '_Nu_',  num2str(Nu), '_v701.mat'];
file_regions = ['regions_quadruple_tank_mpt3_', '_Ny_', num2str(Ny), '_Nu_',  num2str(Nu), '.mat'];
load(file_regions)

%% Simulation Parameters
Tsim = 250;
Nsim = Tsim/Ts;
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

% x(:,1) = [-1.5 1.1 -0.2 0.1]';
x(:,1) = [-2.5 2 -0.5 1.5]';

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
        flag_bad = 1
    end
    
   	u_calc = Regions{index,3}*x(:,i) + Regions{index,4};
    
    u(:,i) = u_calc(1:Ncontrol);   
    x(:,i+1) = Ad*x(:,i) + Bd*u(:,i);
    
    if(i==50)
%         x(:,i+1) = 3*[-1 0.5 -0.2 2.8]';
        x(:,i+1) = [-1 2.5 -0.8 5.0]';
    end

end


% % % vet_index = zeros(1,Nsim);
% % % cont_out = 0;
% % % cont_reg = 0;
% % % vetor_repeticao = zeros(1,Nsim);
% % % 
% % % for i = 1:Nsim
% % %    
% % %     %U Controller
% % %     index = 0;
% % %     y(:,i) = Cd*x(:,i); 
% % %     cont_reg(i) = 0;
% % %     for j = 1:size(Regions,1)
% % %         A_CRi = Regions{j,1};
% % %         b_CRi = Regions{j,2};
% % %         flag = 0;       
% % %         for k = 1:size(A_CRi,1)
% % %             if(((A_CRi(k,:)*x(:,i)) > (b_CRi(k))))
% % %                 flag = 1;
% % %             end
% % %         end
% % %         if flag == 0
% % %             cont_reg(i) = cont_reg(i) + 1;
% % %             index = j;
% % %             if (cont_reg(i) > 1)
% % %                 j
% % %             end
% % %         end
% % %     end
% % %     
% % %     vet_index(i) = index;
% % %     u_calc = Regions{index,3}*x(:,i) + Regions{index,4};
% % %     u(:,i) = u_calc(1:Ncontrol);
% % %     x(:,i+1) = Ad*x(:,i) + Bd*u(:,i);
% % % 
% % %  
% % % end

% % %% Plot
% % %%%%%Plot Estados - X
% % figure
% % plot(t,x(1,1:length(t)))
% % hold on
% % plot(t,x(2,1:length(t)), '-r')
% % plot(t,x(3,1:length(t)))
% % plot(t,x(4,1:length(t)))
% % grid on
% % title('Estados')
% % legend('$x_1$','$x_2$','$x_3$','$x_4$')
% % 
% % %%%%%Plot Saidas - Y
% % % figure
% % % plot(t,y)
% % % grid on
% % % title('Saida - Y')
% % 
% % %%%%%Plot A��es de Controle - U
% % figure
% % plot(t,u(1,:),'-b')
% % hold on
% % plot(t,u(2,:),'-k')
% % grid on
% % title('U')
% % 
% % figure
% % plot(t,vet_index_bst(1,:),'-k')
% % grid on
% % title('Index')

% % % %%
%%
% outputType = 'pdf';
outputType = 'epsc';
%sim('Ex3');
set(groot,'DefaultTextInterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');
set(groot,'defaultAxesFontName', 'Times New Roman');
set(groot,'defaultAxesFontSize', 12);

% % %% State 
% % fig = figure();
% % set(fig,'Visible','off') 
% % 
% % plot(t,x(1,1:length(t)),'Color',[0,0,0],'LineWidth', 0.6)
% % hold on;
% % plot(t,x(2,1:length(t)),'Color',[0.3 0.3 0.3],'LineWidth',0.8)
% % hold on;
% % plot(t,x(3,1:length(t)),'--','Color',[0.55 0.55 0.5],'LineWidth',0.8)
% % hold on;
% % % plot(t,x(4,1:length(t)),'Color',[0.8 0.8 0.8],'LineWidth',0.6)
% % plot(t,x(4,1:length(t)),':','Color',[0.0 0.0 0.0],'LineWidth',0.6)
% % hold on;
% % 
% % h = legend('\bf{$x_1$}','\bf{$x_2$}','\bf{$x_3$}','\bf{$x_4$}');%,'Interpreter','latex','Units','centimeters');
% % set(h,'Interpreter','latex','FontSize',8)
% % % set(h,'Position', [h.Position(1) h.Position(2) 0.03 0.07])
% % % set(h,'Position', [h.Position(1)-0.02 h.Position(2) 0.001 0.07])
% % % set(h,'Position', [h.Position(1) h.Position(2) 0.5*h.Position(3) 2*h.Position(4)])
% % % set(h,'Position', [h.Position(1) h.Position(2) 0.5*h.Position(3) 2*h.Position(4)])
% % % set(h,'Position', [h.Position(1) h.Position(2) 0.5 h.Position(4)])
% % % set(h,'Location','northeast')
% % set(h,'Location','best')
% % 
% % axis([0 Nsim -3 6])
% % set(gca,'FontSize', 8)
% % xlabel('Time [s]','FontSize',8)
% % set(gca,'Ytick',-2.5:2.5:5)
% % set(gca,'Xtick',0:50:Nsim)
% % grid on
% % 
% % set(gca,'LooseInset',get(gca,'TightInset'));
% % set(fig, 'PaperUnits', 'centimeters')
% % set(fig, 'PaperSize', [4 4.25]);
% % set(fig, 'PaperPosition', [0 0 4 4.25]);
% % saveas(fig, 'result_state_bst_original', outputType);
% % 
% % %% Control
% % fig = figure();
% % set(fig,'Visible','off') 
% % 
% % plot(t,u(1,1:length(t)),'Color',[0,0,0],'LineWidth', 0.6)
% % hold on;
% % plot(t,u(2,1:length(t)),'-.','Color',[0.4 0.4 0.4],'LineWidth', 0.8)
% % plot(t,3*ones(1,length(t)),'--','Color',[0 0 0])
% % plot(t,-3*ones(1,length(t)),'--','Color',[0 0 0])
% % 
% % 
% % h = legend('\bf{$u_1$}','\bf{$u_2$}','Interpreter','latex');
% % set(h,'Interpreter','latex','FontSize',8)
% % % set(h,'Position', [0.815 0.60 0.1 0.009])
% % % set(h,'Location','northeast')
% % set(h,'Location','best')
% % 
% % axis([0 Nsim -3.5 3.5])
% % set(gca,'FontSize', 8)
% % xlabel('Time [s]','FontSize',8)
% % set(gca,'Ytick',-3.0:1.5:3.0)
% % set(gca,'Xtick',0:50:Nsim)
% % grid on
% % 
% % set(gca,'LooseInset',get(gca,'TightInset'));
% % set(fig, 'PaperUnits', 'centimeters')
% % set(fig, 'PaperSize', [4 4.25]);
% % set(fig, 'PaperPosition', [0 0 4 4.25]);
% % saveas(fig, 'result_control_bst_original', outputType);

%%%%%%%%%%%%%%%%%%%%%5
%% State 
fig = figure();
set(fig,'Visible','off') 

plot(t,x(1,1:length(t)),'Color',[0,0,0],'LineWidth', 0.8)
hold on;
plot(t,x(2,1:length(t)),'Color',[0.45 0.45 0.45],'LineWidth',1)
hold on;
plot(t,x(3,1:length(t)),'--','Color',[0.7 0.7 0.7],'LineWidth',1)
hold on;
plot(t,x(4,1:length(t)),'--','Color',[0.0 0.0 0.0],'LineWidth',0.8)
hold on;

h = legend('\bf{$x_1$}','\bf{$x_2$}','\bf{$x_3$}','\bf{$x_4$}');%,'Interpreter','latex','Units','centimeters');
set(h,'Interpreter','latex','FontSize',10)
% set(h,'Position', [h.Position(1) h.Position(2) 0.03 0.07])
% set(h,'Position', [h.Position(1)-0.02 h.Position(2) 0.001 0.07])
% set(h,'Position', [h.Position(1) h.Position(2) 0.5*h.Position(3) 2*h.Position(4)])
% set(h,'Position', [h.Position(1) h.Position(2) 0.5*h.Position(3) 2*h.Position(4)])
% set(h,'Position', [h.Position(1) h.Position(2) 0.5 h.Position(4)])
set(h,'Location','northeast')
% set(h,'Location','best')

axis([0 Nsim -3 6])
set(gca,'FontSize', 10)
xlabel('Time [s]','FontSize',10)
set(gca,'Ytick',-2.5:2.5:5)
set(gca,'Xtick',0:50:(Nsim+10))
grid on

set(gca,'LooseInset',get(gca,'TightInset'));
set(fig, 'PaperUnits', 'centimeters')
set(fig, 'PaperSize', [8 4.5]);
set(fig, 'PaperPosition', [0 0 8 4.5]);
% % % % % % % % % % % % saveas(fig, 'result_state_quadtank_bst_original_font10', outputType);
% saveas(fig, 'result_state_quadtank_bst_intersec_font10', outputType);

%% Control
fig = figure();
set(fig,'Visible','off') 

plot(t,u(1,1:length(t)),'Color',[0,0,0],'LineWidth', 1)
hold on;
plot(t,u(2,1:length(t)),'Color',[0.55 0.55 0.55],'LineWidth', 1)
plot(t,3*ones(1,length(t)),'--','Color',[0 0 0],'LineWidth', 0.8)
plot(t,-3*ones(1,length(t)),'--','Color',[0 0 0],'LineWidth', 0.8)


h = legend('\bf{$u_1$}','\bf{$u_2$}','Interpreter','latex');
set(h,'Interpreter','latex','FontSize',10)
% set(h,'Position', [0.815 0.60 0.1 0.009])
set(h,'Location','northeast')
% set(h,'Location','best')

axis([0 Nsim -3.5 3.5])
set(gca,'FontSize', 10)
xlabel('Time [s]','FontSize',10)
set(gca,'Ytick',-3.0:1.5:3.0)
set(gca,'Xtick',0:50:(Nsim+10))
grid on

set(gca,'LooseInset',get(gca,'TightInset'));
set(fig, 'PaperUnits', 'centimeters')
set(fig, 'PaperSize', [8 4.5]);
set(fig, 'PaperPosition', [0 0 8 4.5]);
% % % % % % % % % % % % % % % % saveas(fig, 'result_control_quadtank_bst_original_font10', outputType);
% saveas(fig, 'result_control_quadtank_bst_intersec_font10', outputType);