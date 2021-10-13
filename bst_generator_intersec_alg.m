clear all
% clc
close all
N = 4;
Ny = 5;                                                                                                                                                                                                                                                                                                                                                                             
Nu = 2; 

name_file  = ['regions_double_int_mpt3_', num2str(N), '.mat']; % N = 4;
% name_file  = ['regions_example2_kd_tree_mpt3_', num2str(N), '.mat']; % N = 4, N = 5;
% name_file = ['regions_quad_int_mpt3_', '_Ny_', num2str(Ny), '_Nu_',  num2str(Nu), '.mat'];% N = 6
% name_file = ['regions_quadruple_tank_mpt3_', '_Ny_', num2str(Ny), '_Nu_',  num2str(Nu), '.mat'];% Ny = 5, Nu = 2; Ny = 8, Nu = 2; 
load(name_file)
%% Parameters
it_max = 25*1500;
tol = 1e-6;
tol2 = 1e-8;
Nstate = 2;  %% Update if you changed the system
%% Solver configurations
solver_opt = sdpsettings;
solver_opt.solver = 'linprog';
solver_opt.verbose = 0;
solver_opt.debug = 0;
solver_opt.cachesolvers = 1;
solver_opt.warning = 0;
solver_opt.linprog.TolCon = tol2;
solver_opt.linprog.TolFun = tol2*1e-1;


%% Verify or load all the inequations
ineq_list = verifiy_total_ineq(Regions);

%% Classify regions with original algorithm
tic
chebychev_centers = zeros(size(Regions,1), Nstate);
for i = 1:size(Regions,1)
    chebychev_centers(i,:) = point_in_region_chebyshev(Regions{i,1}, Regions{i,2}, Nstate, solver_opt);
end
time_cheby = toc;

tic
for i = 1:size(ineq_list,1)
    for j = 1:size(Regions,1)
        result = side_ineq_region_intersec(ineq_list(i,:),Regions(j,:), chebychev_centers(j,:)', solver_opt);
        ineq_list{i,6}(j,1) = result;
    end
end
time_ineq = toc;

%% List all the control laws
controls = list_control_laws(Regions, ineq_list, tol);

%%
tic
clear nodes
last_index_ineq = 0;
index_ineq = 1;
unex_node = 1;
nodes{1,1} = ineq_list{1,1};         %aj from dj
nodes{1,2} = ineq_list{1,2};         %bj from dj
nodes{1,3} = [];
nodes{1,4} = (1:size(Regions,1))';     %regions for the node
nodes{1,5} = 2;                     %<= node
nodes{1,6} = 3;                     %>= node
nodes{1,7} = [];                    %parent node
nodes{1,8} = [];                    %parent inequations side

%%
it = 0;
while isempty(unex_node) == 0 && it < it_max 
    it = it + 1;
    
    index_node = unex_node(end,1);
   
    %Define inequation to the node
    
    [index_ineq, num_max, flag_ineq_region] = define_inequation_node_less(nodes, index_node, ineq_list, nodes{index_node,3}, controls, Regions, tol);

    vetor_index(1,it) = index_ineq;

    
    nodes{index_node,1} = ineq_list{index_ineq,1};                   %aj from dj
    nodes{index_node,2} = ineq_list{index_ineq,2};                   %bj from dj
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
    
    index_side = [];
    
    %Number of control laws from the remaing regions
    Regions_remaining = {};
    for i = 1:size(nodes{index_node,4},1)
        Regions_remaining = [Regions_remaining; Regions(nodes{index_node,4}(i,1),:)];
    end
    controls_remaining = simplified_list_control_laws(Regions_remaining,tol);
    
    
    if size(controls_remaining,1) < 5
        for i = 1:size(nodes{index_node,4},1)
            index_side(i) = ineq_list{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1);
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
        if ineq_list{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1) == 1 || ((ineq_list{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1) == 3) && (flag_greater_side == 1))
            nodes{nodes{index_node,5},4} = [nodes{nodes{index_node,5},4}; nodes{index_node,4}(i,1)];
            nodes{nodes{index_node,5},8} = [nodes{index_node,8}; 1];                                %Save inequation side
        end
        if ineq_list{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1) == 2 || ((ineq_list{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1) == 3) && (flag_less_side == 1))
            nodes{nodes{index_node,6},4} = [nodes{nodes{index_node,6},4}; nodes{index_node,4}(i,1)];
            nodes{nodes{index_node,6},8} = [nodes{index_node,8}; 2];                                %Save inequation side
        end
    end
    
    

    unex_node(end,:) = [];
    
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
time_bst = toc;

[nodes_out, list_depth, number_leafs, max_depth] = count_depth_leafs(nodes);
fprintf('Time Chebyshev: %.3f   Time Inequations: %.3f   Time BST: %.3f   Total Time: %.3f\nNumber Ineq: %d   Number Nodes: %d\nNumber Leaf: %d   Depth: %d\n\n', time_cheby, time_ineq, time_bst, time_ineq + time_bst+time_cheby, size(ineq_list,1), size(nodes,1), number_leafs, max_depth);

%%


% name_file = ['bst_intersec_double_int_mpt3_', num2str(N), '.mat'];
% name_file = ['bst_intersec_example2_kd_tree_mpt3_', num2str(N), '.mat'];
% name_file = ['bst_intersec_quad_integrator_mpt3_', '_Ny_', num2str(Ny), '_Nu_',  num2str(Nu), '.mat'];
% name_file = ['bst_intersec_quadruple_tank_mpt3_', '_Ny_', num2str(Ny), '_Nu_',  num2str(Nu), '.mat'];

% % save(name_file,'nodes')