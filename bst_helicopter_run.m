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
Nout = size(Cd,1);
Ts = 0.01;
Ny = 5;
Nu = 2;
%file_bst = ['bst_original_helicopter_mpt3_', '_Ny_', num2str(Ny), '_Nu_',  num2str(Nu), '_v01.mat'];
% file_bst = ['bst_original_helicopter_mpt3_', '_Ny_', num2str(Ny), '_Nu_',  num2str(Nu), '_v11.mat'];
file_bst = ['bst_intersec_helicopter_mpt3_', '_Ny_', num2str(Ny), '_Nu_',  num2str(Nu), '_v11.mat'];
load(file_bst)
%%file_regions = ['regions_helicopter_mpt3_', '_Ny_', num2str(Ny), '_Nu_',  num2str(Nu), '_v01.mat'];
file_regions = ['regions_helicopter_mpt3_', '_Ny_', num2str(Ny), '_Nu_',  num2str(Nu), '_v11.mat'];
load(file_regions)

%% Simulation Parameters
Nsim = 1000;
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

x(:,1) = [0.5 0.5 0 0 0 0]';


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

%% Plot
%%%%%Plot Estados - X
figure
plot(t,x(1,1:length(t)))
hold on
plot(t,x(2,1:length(t)), '-r')
plot(t,x(3,1:length(t)))
plot(t,x(4,1:length(t)))
plot(t,x(5,1:length(t)))
plot(t,x(6,1:length(t)))
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
hold on
plot(t,u(2,:),'-k')
grid on
title('U')


% % % %%
% % % % outputType = 'pdf';
% % % outputType = 'epsc';
% % % %sim('Ex3');
% % % set(groot,'DefaultTextInterpreter','latex');
% % % set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
% % % set(groot, 'defaultLegendInterpreter','latex');
% % % set(groot,'defaultAxesFontName', 'Times New Roman');
% % % set(groot,'defaultAxesFontSize', 8);
% % % 
% % % %% State 
% % % fig = figure();
% % % set(fig,'Visible','off') 
% % % 
% % % plot(t,x(1,1:length(t)),'Color',[0,0,0],'LineWidth', 1.5)
% % % hold on;
% % % plot(t,x(2,1:length(t)),'--','Color',[0.35,0.35,0.35],'LineWidth', 1)
% % % hold on;
% % % 
% % % h = legend('\bf{$x_1$}','\bf{$x_2$}','Interpreter','latex');
% % % set(h,'Interpreter','latex','FontSize',50)
% % % % set(h,'Position', [0.815 0.60 0.1 0.009])
% % % set(h,'Location','northeast')
% % % 
% % % axis([0 (Nsim-1) -2 4])
% % % set(gca,'FontSize', 40)
% % % xlabel('Time [s]','FontSize',40)
% % % set(gca,'Ytick',-2:2:4)
% % % set(gca,'Xtick',0:5:(Nsim-1))
% % % grid on
% % % 
% % % set(gca,'LooseInset',get(gca,'TightInset'));
% % % set(fig, 'PaperUnits', 'centimeters')
% % % set(fig, 'PaperSize', [17.5 13]);
% % % set(fig, 'PaperPosition', [0 0 17.5 13]);
% % % %saveas(fig, 'D:\Documentos\OneDrive - UDESC Universidade do Estado de Santa Catarina\TCC\Artigo BST\figuras\resultados\result_state_bst_original_v2', outputType);
% % % % saveas(fig, 'resultados/result_state_bst_original_v100', outputType);
% % % saveas(fig, 'resultados/result_state_bst_intersec_v100', outputType);
% % % 
% % % %% Control
% % % fig = figure();
% % % set(fig,'Visible','off') 
% % % 
% % % plot(t,u(1,1:length(t)),'Color',[0,0,0],'LineWidth', 1.5)
% % % hold on;
% % % plot(t,Umax*ones(1,length(t)),'--','Color',[0 0 0])
% % % plot(t,Umin*ones(1,length(t)),'--','Color',[0 0 0])
% % % 
% % % 
% % % h = legend('\bf{$u$}');
% % % set(h,'Interpreter','latex','FontSize',50)
% % % % set(h,'Position', [0.815 0.60 0.1 0.009])
% % % set(h,'Location','northeast')
% % % 
% % % axis([0 (Nsim-1) -0.6 0.6])
% % % set(gca,'FontSize', 40)
% % % xlabel('Time [s]','FontSize',40)
% % % set(gca,'Ytick',-0.5:0.5:0.5)
% % % set(gca,'Xtick',0:5:(Nsim-1))
% % % grid on
% % % 
% % % set(gca,'LooseInset',get(gca,'TightInset'));
% % % set(fig, 'PaperUnits', 'centimeters')
% % % set(fig, 'PaperSize', [17.5 13]);
% % % set(fig, 'PaperPosition', [0 0 17.5 13]);
% % % %saveas(fig, 'D:\Documentos\OneDrive - UDESC Universidade do Estado de Santa Catarina\TCC\Artigo BST\figuras\resultados\result_control_bst_original_v2', outputType);
% % % % saveas(fig, 'resultados/result_control_bst_original_v100', outputType);
% % % saveas(fig, 'resultados/result_control_bst_intersec_v100', outputType);
