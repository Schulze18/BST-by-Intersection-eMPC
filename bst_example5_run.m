close all

%% Model
Ad = [4 -1.5 0.5 -0.25;
     4 0 0 0;
     0 2 0 0;
     0 0 0.5 0];
Bd = [0.5 0 0 0]';
Cd = [0.083 0.22 0.11 0.02];
Ts = 1;

Nstate = size(Ad,1);
Ncontrol = size(Bd,2);
Nout = size(Cd,1);
Nref = Nout;
 
 
Nsim = 50;
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

x(:,1) = [0.1 0.1 -0.3 1]*2'; 
for i = 1:Nsim
    i;
    index = 0;
    y(:,i) = Cd*x(:,i); 
%     erro(:,i) = ) - y(:,i); 
    cont_reg(i) = 0;
    
    num_operation = 0;
% %     
% %     for j = 1:size(Regions,1)
% %         A_CRi = Regions{j,1};
% %         b_CRi = Regions{j,2};
% %         flag = 0;       
% %         for k = 1:size(A_CRi,1)
% % %             if isnan((A_CRi(k,:)*[x(:,i); u(:,i-1); ref(:,i)]))
% % %                 i
% % %                 disp('bad');
% % %             end
% % 
% %             num_operation = num_operation + 1;
% % 
% %             if(((A_CRi(k,:)*[x(:,i); u(:,i-1); ref(:,i)]) > (b_CRi(k))))% || ( (isnan((A_CRi(k,:)*[x(:,i); u(:,i-1); ref(:,i)])))==1))
% %                 flag = 1;
% %             end
% %         end
% %         if flag == 0
% %             cont_reg(i) = cont_reg(i) + 1;
% %             index = j;
% %             if (cont_reg(i) > 1)
% %                 j
% %             end
% %         end
% %     end
% %    
    
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
%     u(:,i) = u(:,i-1) + deltau(:,i);
    
    x(:,i+1) = Ad*x(:,i) + Bd*u(:,i);% + 0.1*[rand(1)-rand(1) ; rand(1)-rand(1)];

end

%%
%%%%%Plot Saidas - Y
figure
plot(t,y)
title('Saida - Y')

%%%%%Plot A��es de Controle - U
figure
plot(t,u(1,:),'-b')
title('U')
