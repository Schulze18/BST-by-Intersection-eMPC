clear all
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
P = Q;%zeros(4);

%%
Nstate = size(Ad,1);
Ncontrol = size(Bd,2);
Nout = size(Cd,1);
Nref = 0;%Nout;

%% Constraints
Umax = 1;
Umin = -1;
Xmax = [];
Xmin = [];
Ymax = 10;
Ymin = -10;

%% Horizons
Ny = 3;
Nu = 3;

%% Matrices 
[H, F, Sx, Su, Clinha] = regulator_matrices_cost_function(Ad, Bd, Cd, P, Q, R, Ny, Nu);

[G, W, E, S] = regulator_constraints_matrices(Bd, Clinha, Su, Sx, H, F, Nu, Ny, Umax, Umin, Xmax, Xmin, Ymax, Ymin);

%%
% Code parameters
tol = 1e-6;
n_plot = 100;
last_plot = 0;
num_Gu = 100;
%Solver Options
sdp_opt = sdpsettings;
sdp_opt.solver = 'sdpt3';
sdp_opt.verbose = 0;
sdp_opt.cachesolvers = 1;
sdp_opt.sdpt3.maxit = 30;
sdp_opt.sdpt3.steptol = 1.0000e-7;
sdp_opt.sdpt3.gaptol = 5.000e-7;
 
 %% Loop variables Initialization
Lopt{1,1} = [];Lcand{1,2} = []; Lcand{1,3} = [];Lcand{1,4} = [];
%Lcand{1,1} = [6]; Lcand{1,2} = []; Lcand{1,3} = [];
%Lcand{1,1} = [1,2,3,4]'; Lcand{1,2} = []; Lcand{1,3} = [];
%Lopt{1,1} = [1,2,3,4,5];
Regions = {};
N_zeros = 0;
comb_ruim = {};
n=0;
fprintf('Exploradas: %d\nInexploradas: %d\nRegioes: %d\n\n',0,1,0);
tic
while isempty(Lcand) == 0
    %fprintf('Exploradas: %d\nInexploradas: %d\nRegioes: %d\n\n',length(Lopt),length(Lcand),length(Regions));
    n = n+1;
%     if n == 22
%         disp('hue')
%     end
    status_infesiable = 0;
    flag_verified = 0;
    flag_x_nan = 0;
    %%%%%% Alteracao
    index = Lcand{end,1};
    if isempty(Lcand{end,1}) == 0
        
        %Verify if the index was already tested
        if check_if_verified(Lcand{end,1}, Lopt) == 1
            flag_verified = 1;
            %status_infesiable = 1;
        else
            flag_verified = 0;
            %status_infesiable = 0;
            [G_tio, W_tio, S_tio] = build_active_const(G, W, S, Lcand{end,1});
        end

        if (rank(G_tio) < size(G_tio,1)) && (rank(G_tio) < size(G_tio,2)  &&  flag_verified == 0)%status_infesiable == 0)% && (size(Lcand{end,1},1) > 1)
            [G_tio, W_tio, S_tio, index, status_infesiable, flag_x_nan] = resolve_degenerancy(G, W, S, H, F, Lcand{end,2}, Lcand{end,3}, Nstate, Ncontrol, Nout, Ny, Nu, Lcand{end,1}, tol, Lcand{end,4}, sdp_opt);
            %disp('Degen');
            index = sort(index);
            if isequal(index,Lcand{end,1})
                disp('ruim degen')
            end
            
            
            %Verify if the resulting index was already tested
            if ((check_if_verified(index, Lopt) == 1) && (status_infesiable == 0))
                %status_infesiable = 1;
                Lopt{end+1,1} = sort(Lcand{end,1});
                flag_verified = 1;
            end
            
            %Add the resulting index to the optimized list
            if ((isequal(index,Lcand{end,1})==0) && (isempty(index)==0) && (status_infesiable == 0) &&  (flag_verified == 0))
                if (n>1)
                    %Lopt{end+1,1} = sort(index);
                    Lopt{end+1,1} = index;
                else
                    %Lopt{1,1} = sort(index);
                    Lopt{1,1} = index;
                end
            end
            
        end
        
        if  (status_infesiable == 0) && (flag_verified == 0) 
            [A, b, type, origem] = define_region(G, W, S, G_tio, W_tio, S_tio, H, tol);
            if ((sum((sum(isnan(A)))) == 1) || (sum((sum(isinf(A)))) == 1))
               disp('bad') 
            end
            if ((isempty(A) == 0) && (sum((sum(isnan(A)))) == 0) && (sum((sum(isinf(A)))) == 0)) %Soma de todos os elementos de isnan(A)
              
                A_old_test = A;
                b_old_test = b;
               [A, b, type, origem] =  remove_redundant_constraints(A, b, type, origem, Nu, Nstate, sdp_opt);
               if ((size(A_old_test,1) - size(A,1)) > 0)
                    A_old_test;
                    A;
               end
%                 Lcand{end,2} = A;
%                 Lcand{end,3} = b;
                [Kx, Ku] = define_control(G, W, S, G_tio, W_tio, S_tio, H, F, Ncontrol);
            end           
        end
        %Lopt = [Lopt; sort(Lcand{end,1})];
        if (n>1 && flag_verified == 0)
            Lopt{end+1,1} = sort(Lcand{end,1});
        %else
        elseif flag_verified == 0
            Lopt{1,1} = sort(Lcand{end,1});
        end
    else
        A = -S;
        b = W;
        origem = (1:size(G,1))';
        type = ones(size(G,1),1);
        %type(5:end) = 2;
        [A, b, type, origem] = remove_redundant_constraints(A, b, type, origem, Nu, Nstate, sdp_opt);
        Kx = (-inv(H)*F');
        Kx = Kx(1:Ncontrol,:);
        Ku = zeros(Ncontrol,1);
% % %         Lopt = [Lopt; []];
% %         Lopt{end+1,1} = [];
        if (n>1)
             Lopt{end+1,1} = [];
        else
             Lopt{1,1} = [];
        end
    end
    
    L_new_cand = {};
    if ((isempty(A) == 0) && (status_infesiable == 0) && (sum((sum(isnan(A)))) == 0) && (sum((sum(isinf(A)))) == 0) && (flag_verified == 0))
        
        CR = {A b Kx Ku n Lcand{end,2} Lcand{end,3} flag_x_nan status_infesiable};
        Regions = [Regions; CR];
        
        L_new_cand = {};
        for i = 1:size(A,1)
            possible_new_set = [];
            flag_repeticao = 0;
            %%%%%%%%indices_cand = Lcand{end,1}; 
            indices_cand = index;
            
            %%%%%%%%%%%%%%%%%%
            for j = 1:length(indices_cand)
                if (indices_cand(j) == origem(i) && type(i) == 1)
                    flag_repeticao = 1;
                end
            end
            %%%%%%%%%%%%%%%%%%
            new_index = [];
            %if (type(i) == 1 ) && (origem(i) ~= 0) && (isempty(Lcand{end,1}) || (flag_repeticao == 0))
            if (type(i) == 1 ) && (origem(i) ~= 0 && origem(i) <= num_Gu) && (isempty(Lcand{end,1}) || (flag_repeticao == 0))   
                %%%%%%%%%%%%%%%possible_new_set = [Lcand{end,1}; origem(i)];
                possible_new_set = [index; origem(i)];
                %new_index = origem(i); %Old
                new_index = [Lcand{end,4}; origem(i)];%Only for test - saving the latest index add
                
            %elseif type(i) == 2 && origem(i) ~= 0
            %elseif (type(i) == 2) && (origem(i) ~= 0) && (origem(i) > num_Gu)
            %elseif (type(i) == 2) && (isempty(Lcand{end,1}) || (flag_repeticao == 0))
            %elseif (type(i) == 2 || origem(i) > num_Gu) && (isempty(Lcand{end,1}) || (flag_repeticao == 0))
            
            %elseif (type(i) == 2 || origem(i) > num_Gu) && (isempty(Lcand{end,1})==0)
            elseif (type(i) == 2) && (isempty(Lcand{end,1})==0)
                 %%%%%%%%%%%%%%%%%%%%%%%possible_new_set = Lcand{end,1};
                 possible_new_set = index;
                 
                 %possible_new_set = possible_new_set(1:end-1,1);
                 possible_new_set(origem(i)) = [];
                  
                 %Only for test - saving the latest index add
                 new_index = Lcand{end,4};
            else
                if (Lcand{end,1} == origem(i))
                    comb_ruim{end+1} = [Lcand{end,1}; origem(i)];
                end
                N_zeros = N_zeros+1;
            end

            %possible_new_set
            possible_new_set = sort(possible_new_set);

            %if (check_if_verified(possible_new_set,Lopt) == false) && (isempty(possible_new_set) == 0)
            if ((check_if_verified(possible_new_set,Lopt) == false) && (isempty(L_new_cand) || check_if_verified(possible_new_set,L_new_cand) == false) && (check_if_verified(possible_new_set,Lcand) == false))
                 L_new_cand{end+1,1} = possible_new_set;
                 L_new_cand{end,2} = A;
                 L_new_cand{end,3} = b;
                 L_new_cand{end,4} = new_index;
%                  disp('hue');
                  %L_new_cand = {L_new_cand; possible_new_set}
            end
        %L_new_cand
        end
    else
        L_new_cand = {};
    end
    
    Lcand(end,:) = [];
    if isempty(L_new_cand) == 0
        if isempty(Lcand) == 1
            Lcand = L_new_cand;
        else
            Lcand = [Lcand; L_new_cand];
        end
    end
%     Lcand{end,1}
    
    if (length(Lopt) - last_plot) > n_plot
        fprintf('Exploradas: %d\nInexploradas: %d\nRegioes: %d\n\n',length(Lopt),length(Lcand),length(Regions));
        last_plot = length(Lopt);
    end
end
toc
fprintf('Exploradas: %d\nInexploradas: %d\nRegioes: %d\n\n',length(Lopt),length(Lcand),length(Regions));
figure