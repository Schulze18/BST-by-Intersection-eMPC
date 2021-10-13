clear all
% clc
tol = 1e-6;

%%
% State Space Model
Ad = [0.7326 -0.0861;
      0.1722 0.9909];
Bd = [0.0609 0.0064]';
Cd = [0 1.4142];
Nstate = size(Ad,1);
Ncontrol = size(Bd,2);
Nout = size(Cd,1);
Nref = 0;%Nout;
Ny = 5;
Nu = 5;
Nc = 5;

%% Load Regions
load('regions_exemplo_tcc.mat');


%% Step 1 - Algorithm 2 - Computation all the I(j+) and I(j-)
%% Verify or load all the inequations
ineq_pos = verifiy_total_ineq(Regions);
% load('File_results/ineq_dj_10_3_u2.mat')
% load('File_results/ineq_10_3_u_du_v_opt_test');   
% load('/File_results/ineq_10_2_u3_du3.mat');
% load('File_results/ineq_10_2_u_all_v2.mat');
ineq_neg = cellfun(@(x) x*(-1),ineq_pos,'un',0);

% load('File_results/chebs_center_10_2_u_all_v2.mat');

%%
ineq_test = which_region_ineq(ineq_pos,Regions,tol);

%%
%Solver configurations
solver_opt = sdpsettings;
solver_opt.solver = 'glpk';
solver_opt.verbose = 0;
solver_opt.debug = 0;
solver_opt.cachesolvers = 1;
solver_opt.warning = 0;
% solver_opt.sdpt3.maxit = 20;
% solver_opt.sdpt3.steptol = 1.0000e-07;
% solver_opt.sdpt3.gaptol = 5.000e-7;
% solver_opt.sedumi.eps = 1.0000e-08;
% solver_opt.sedumi.maxiter = 100;
% solver_opt.quadprog.MaxIter = 50;
% solver_opt.removeequalities = -1;
% solver_opt.saveduals = 0;

%% Constraints
Umax = 1.5;
Umin = -1.5;
Xmax = [];
Xmin = [];
Ymax = [];%10;
Ymin = [];%-10;
%% Cost Function Weight
Q = eye(2);%diag([5 10 10 10]);
R = 0.01;
P = Q;%zeros(4);
%% Matrices 
% % [H, F, Sx, Su, Clinha] = regulator_matrices_cost_function(Ad, Bd, Cd, P, Q, R, Ny, Nu);
% % 
% % % [G, W, E, S] = regulator_constraints_matrices(Bd, Clinha, Su, Sx, H, F, Nu, Ny, Umax, Umin, Xmax, Xmin, Ymax, Ymin);
% % % G = blkdiag(eye(Nc*Ncontrol))%,zeros((Nu-Nc)*Ncontrol)); 
% % G = [eye(Nc*Ncontrol) zeros((Nu-Nc)*Ncontrol)];
% % Wumax = Umax;
% % Wumin = -Umin;
% % for i = 1:(Nc-1)
% %     Wumax = [Wumax; Umax];
% %     Wumin = [Wumin; -Umin];
% % end
% % G = [G; -G];
% % W = [Wumax; Wumin];
% % E = [zeros(Nc*Ncontrol,Nstate); zeros(Nc*Ncontrol,Nstate)];
% % S = E + G*inv(H)*F';



%% Classify regions by intersection
% chebychev_centers = zeros(size(Regions,1),Nstate);
% chebychev_centers = zeros(size(Regions,1), Nstate);
% for i = 1:size(Regions,1)
%     %chebychev_centers(end+1,:) = chebychev_ball(Regions{i,1}, Regions{i,2}, [], [], [], [], [], 16, 4, 4, 10, 2, solver_opt);
% %     chebychev_centers(i,:) = chebychev_ball(Regions{i,1}, Regions{i,2}, G, W, S, H, F, 16, 4, 4, 10, 2, solver_opt);
%     chebychev_centers(i,:) = chebychev_ball(Regions{i,1}, Regions{i,2}, G, W, S, H, F, Nstate, Ncontrol, Nout, Ny, Nu, solver_opt);
% %     chebychev_centers(i,:) = chebychev_ball(Regions{i,1}, Regions{i,2}, G, W, S, H, F, Nstate, Ncontrol, Nout, Ny, Nu, solver_opt);
% end

%
% solver_opt.sedumi.maxiter = 1;
% tic
% for i = 1:size(ineq_pos,1)
%     for j = 1:size(Regions,1)
% %         j
% %         if j == 2
% %             disp('hue')
% %         end
%         result2 = side_ineq_region_intersec(ineq_pos(i,:),Regions(j,:), chebychev_centers(j,:)', solver_opt);
%         ineq_pos{i,8}(j,1) = result2;
% %         ineq_pos{i,7}(j,1) = result2;
% %         result = side_ineq_region_intersec_feasible(ineq_pos(i,:),Regions(j,:), chebychev_centers(j,:)');
% %         ineq_pos{i,8}(j,1) = result;
%     end
% %     toc
% end
% ineq_pos(:,6) = ineq_pos(:,8);

%% Classify regions with original algorithm

warm_vec = zeros(size(Regions,1),size(Regions{1,1},2));
% solver_opt.usex0 = 1;

tic
for i = 1:size(ineq_pos,1)
    %i
    for j = 1:size(Regions,1)
        result = side_ineq_region(ineq_pos(i,:),Regions(j,:), solver_opt);
%         [result, warm_value] = side_ineq_region_warm(ineq_pos(i,:),Regions(j,:), solver_opt, warm_vec(j,:)');
        ineq_pos{i,6}(j,1) = result;
%         warm_vec(j,:) = warm_value';
     %   delta_time = toc - time_old
      %  time_old = toc;
    end
%     toc
end
% toc

%% List all the control laws
controls = list_control_laws(Regions, ineq_pos, tol);

%%
% tic
clear nodes
last_index_ineq = 0;
index_ineq = 1;
unex_node = 1;
nodes{1,1} = ineq_pos{1,1};         %aj from dj
nodes{1,2} = ineq_pos{1,2};         %bj from dj
nodes{1,3} = [];
nodes{1,4} = (1:size(Regions,1))';     %regions for the node
nodes{1,5} = 2;                     %<= node
nodes{1,6} = 3;                     %>= node
nodes{1,7} = [];                    %parent node
nodes{1,8} = [];                    %parent inequations side
%%
% nodes(nodes{1,5},:) = cell(1,7);%[]; nodes{nodes{1,5},2} = []; nodes{nodes{1,5},3} = []; nodes{nodes{1,5},4} = []; nodes{nodes{1,5},5} = []; nodes{nodes{1,5},6} = []; nodes{nodes{1,5},7} = 1;
%     nodes(nodes{1,6},:) = cell(1,7);%[]; nodes{nodes{1,6},2} = []; nodes{nodes{1,6},3} = []; nodes{nodes{1,6},4} = []; nodes{nodes{1,6},5} = []; nodes{nodes{1,6},6} = []; nodes{nodes{1,6},7} = 1;
%     for i = 1:size(nodes{1,4},1)
%         if ineq_pos{nodes{1,3}(end,1),6}(i,1) == 1 || ineq_pos{nodes{1,3}(end,1),6}(i,1) == 3
%             nodes{nodes{1,5},4} = [nodes{nodes{1,5},4}; i];
%         end
%         if ineq_pos{nodes{1,3}(end,1),6}(i,1) == 2 || ineq_pos{nodes{1,3}(end,1),6}(i,1) == 3
%             nodes{nodes{1,6},4} = [nodes{nodes{1,6},4}; i];
%         end
%     end
% 
%     if size(nodes{nodes{1,5},4},1) > 1
%         unex_node = [unex_node; nodes{1,5}]; 
%     end
%     if size(nodes{nodes{1,6},4},1) > 1
%         unex_node = [unex_node; nodes{1,6}]; 
%     end

%%

it_max = 25*1500;
it = 0;
while isempty(unex_node) == 0 && it < it_max 
    it = it + 1;
% %     if it > 40
% %         disp('test')
% %     end
    
    index_node = unex_node(end,1);
    
%     if index_node == 50
%         disp('test')
%     end
    
    
    %Define inequation to the node
    
    %%index_ineq = define_inequation_node(nodes, index_node, ineq_pos, nodes{index_node,3}, controls);
    [index_ineq, num_max, flag_ineq_region] = define_inequation_node_less(nodes, index_node, ineq_pos, nodes{index_node,3}, controls, Regions, tol);
    %index_ineq = define_inequation_node_old(nodes, index_node, ineq_pos);
    vetor_index(1,it) = index_ineq;
    %%vetor_index(2,it) = num_max;
    %%vetor_index(3,it) = flag_ineq_region;
%     flag_new_ineq = 0;
%     for i = 1:size(nodes{index_node,4},1)
%         if flag_new_ineq == 1
%                 break
%         end
%         for j = 1:size(ineq_pos,1)
%             if flag_new_ineq == 1
%                 break
%             end
%             %Verify inequation "not used" that define a Region 
%             for k = 1:size(ineq_pos{j,3},1)
%                 if (ineq_pos{j,3}(k,1) == nodes{index_node,4}(i,1)) && (ismember(j,nodes{index_node,3}) == 0)%j ~= last_index_ineq
%                     last_index_ineq = index_ineq;
%                     index_ineq = j;
%                     flag_new_ineq = 1;
%                     break
%                 end
%             end
%         end
%     end
    
    
    nodes{index_node,1} = ineq_pos{index_ineq,1};                   %aj from dj
    nodes{index_node,2} = ineq_pos{index_ineq,2};                   %bj from dj
    nodes{index_node,3} = [nodes{index_node,3}; index_ineq];        %inequation index
    
    nodes(end+1,:) = cell(1,8);
    nodes(end+1,:) = cell(1,8);
   
    nodes{index_node,5} = size(nodes,1) - 1;             %<= node   
    nodes{index_node,6} = size(nodes,1);                 %>= node
    
    %Initialize node
    nodes(nodes{index_node,5},:) = cell(1,8);
    nodes(nodes{index_node,6},:) = cell(1,8);
    
    %Save parent node
    nodes{nodes{index_node,5},7} =  index_node;
    nodes{nodes{index_node,6},7} =  index_node;
    
    %Save inequations from the branch
    nodes{nodes{index_node,5},3} = [nodes{index_node,3}];
    nodes{nodes{index_node,6},3} = [nodes{index_node,3}];
    
    %Verify if the region in this node are in <= or >= side
% %     for i = 1:size(nodes{index_node,4},1)
% %         if ineq_pos{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1) == 1 || ineq_pos{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1) == 3
% %             nodes{nodes{index_node,5},4} = [nodes{nodes{index_node,5},4}; nodes{index_node,4}(i,1)];
% %         end
% %         if ineq_pos{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1) == 2 || ineq_pos{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1) == 3
% %             nodes{nodes{index_node,6},4} = [nodes{nodes{index_node,6},4}; nodes{index_node,4}(i,1)];
% %         end
% %     end

    %Verify if the region in this node are in <= or >= side
    
    index_side = [];
    
    %Number of control laws from the remaing regions
    Regions_remaining = {};
    for i = 1:size(nodes{index_node,4},1)
        Regions_remaining = [Regions_remaining; Regions(nodes{index_node,4}(i,1),:)];
    end
    %%controls_remaining = list_control_laws(Regions_remaining, ineq_pos, tol);
    controls_remaining = simplified_list_control_laws(Regions_remaining,tol);
    
    
%     controls_remaining = zeros(4); %%%%%%%
%     if size(nodes{index_node,4},1) < 4
    if size(controls_remaining,1) < 5
        for i = 1:size(nodes{index_node,4},1)
            index_side(i) = ineq_pos{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1);
        end
        flag_less_side = ismember(1,index_side);
        flag_greater_side = ismember(2,index_side);
        
        if (flag_less_side == 0) && (flag_greater_side == 0)
            flag_less_side = 1;
            flag_greater_side = 1;
        end
%     end
    else
        flag_less_side = 1;
        flag_greater_side = 1;
    end    

    for i = 1:size(nodes{index_node,4},1)
        if ineq_pos{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1) == 1 || ((ineq_pos{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1) == 3) && (flag_greater_side == 1))
            nodes{nodes{index_node,5},4} = [nodes{nodes{index_node,5},4}; nodes{index_node,4}(i,1)];
            nodes{nodes{index_node,5},8} = [nodes{index_node,8}; 1];                                %Save inequation side
        end
        if ineq_pos{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1) == 2 || ((ineq_pos{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1) == 3) && (flag_less_side == 1))
            nodes{nodes{index_node,6},4} = [nodes{nodes{index_node,6},4}; nodes{index_node,4}(i,1)];
            nodes{nodes{index_node,6},8} = [nodes{index_node,8}; 2];                                %Save inequation side
        end
    end
    
    

    unex_node(end,:) = [];
    %Add new unexplored Nodes
    
%     if (size(nodes{nodes{index_node,5},4},1) > 1) 
%         unex_node = [unex_node; nodes{index_node,5}]; 
%     end
%     if size(nodes{nodes{index_node,6},4},1) > 1
%         unex_node = [unex_node; nodes{index_node,6}]; 
%     end
    
    flag_new_node = 1;
    if (size(nodes{nodes{index_node,5},4},1) > 1)       %Only if rest more than 1 control
        for i = 1:size(controls,1)                      %Only if rest more than 1 control
            if ismember(nodes{nodes{index_node,5},4},[controls{i,3}; controls{i,5}]) == 1
                flag_new_node = 0;
            end
        end
    else
        flag_new_node = 0;
    end
    if flag_new_node 
        unex_node = [unex_node; nodes{index_node,5}]; 
    end
    
    flag_new_node = 1;
    if (size(nodes{nodes{index_node,6},4},1) > 1)       %Only if rest more than 1 control
        for i = 1:size(controls,1)                      %Only if rest more than 1 control
            if ismember(nodes{nodes{index_node,6},4},[controls{i,3}; controls{i,5}]) == 1
                flag_new_node = 0;
            end
        end
    else
        flag_new_node = 0;
    end
    if flag_new_node 
        unex_node = [unex_node; nodes{index_node,6}]; 
    end
    
end
toc

% % %% 
% % vetor_rep_controls = zeros(size(Regions,1),3);
% % for i = 1:size(Regions,1)
% %     Kx = Regions(i,3);
% %     Kc = Regions(i,4);
% %     
% %     for j = 1:size(Regions,1)
% %         Kx_test = Regions(j,3);
% %         Kc_test = Regions(j,4);
% %         
% %         if (isequal(Kx_test,Kx) && isequal(Kc,Kc_test))
% %             vetor_rep_controls(i,1) = vetor_rep_controls(i,1) + 1;
% %             
% %             if (vetor_rep_controls(i,2) == 0 && i~=j)
% %                 vetor_rep_controls(i,2) = j;
% %             elseif (vetor_rep_controls(i,2) > 0 && i~=j)
% %                 vetor_rep_controls(i,3) = j;
% %             end
% %             
% %         end
% %     end
% % end
% % 
% % %%
% % for i = 1:size(control_law,1)
% %     if control_law{i}{4} > 1
% %         i
% %     end
% % end
% % 
% % 
% % %%
% % vetor_rep_ineq = zeros(size(ineq_pos,1),1);
% % for i = 1:size(ineq_pos,1)
% %     for j = 1:size(Regions,1);
% %         A_region = Regions{j,1};
% %         b_region = Regions{j,2};
% %         %test_matrix = [A_regions b_regions; ineq_pos{i,1} ineq_pos{i,2}];
% %         %if rank(test_matrix) == size(Regions,1)
% % %         if ismember([ineq_pos{i,1} ineq_pos{i,2}],[A_regions b_regions]) 
% % %             vetor_rep_ineq(i) = vetor_rep_ineq(i) + 1;
% % %         end
% %         
% %         for k = 1:size(A_region,1)
% %             %Check linear dependecy between the rows
% %             test_matrix = [A_region(k,:) b_region(k,:); ineq_pos{i,1} ineq_pos{i,2}];
% %             rank_test = rank(test_matrix);
% %             
% %             if rank_test == 1
% %                 vetor_rep_ineq(i) = vetor_rep_ineq(i) + 1;
% %             end
% %             
% %             %Check linear dependecy and if both are j+ or j-
% % %             if ((rank_test == 1) && (sum(test_matrix(1,:))/sum(test_matrix(2,:)) >= 0)) == 1
% % %                 %Count number of inequations from the set that are
% % %                 %in the region description
% % %                 vetor_rep_ineq(i) = vetor_rep_ineq(i) + 1;
% % %                 %flag = 0;
% % %                 %break
% % %             end
% %         end
% %         
% %         
% %         
% %     end
% % end
% % 
