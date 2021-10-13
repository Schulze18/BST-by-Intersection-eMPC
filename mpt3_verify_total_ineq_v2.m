function [ineq] = mpt3_verify_total_ineq(optimizer_mpt3_in, r_coef)
 % List all hyperplanes the same way as the MPT3 toolbox
    %% set data
	% all sets must be in minimal H-representation
	optimizer_mpt3_in.Set.minHRep();
	% always operate on a copy of the input polyunion
	optimizer_mpt3.Set = optimizer_mpt3_in.Set.copy();
	% normalize (work with the copy)
% 	optimizer_mpt3.Set.normalize();
 
    
    %%
    H = cat(1, optimizer_mpt3.Set.H);
    H_raw = cat(1, optimizer_mpt3.Set.H);
    %find unique "a" from a'*x<=0 and a'*x>=0
    H = round(H*10^r_coef)/10^r_coef;
    nH_orig = size(H, 1);
    H = unique(H, 'rows');
%     H = uniquetol(H, 'ByRows',1e-4);
    [i, j] = ismember(H, -H, 'rows');
    idx1 = find(i==0);
    idx2 = j(j > (1:length(j))');
    H = H(unique([idx1; idx2]), :);
    H_unique = H_raw(unique([idx1; idx2]), :);
    ineq = {};
    for k = 1:size(H_unique,1)
        ineq{k,1} = H_unique(k,1:(end-1));
        ineq{k,2} = H_unique(k,end);
    end
  
    %%
    ineq_raw_list = [];
    for k = 1:size(optimizer_mpt3.Set,1)
        num_ineq = size(optimizer_mpt3.Set(k).A,1);
        ineq_raw_list = [ineq_raw_list; ones(num_ineq,1)*k];
    end
        index_ineq = {};
    for i = 1:size(H_unique,1)
        flag_index = 0;
        for j = 1:size(ineq_raw_list,1)
%             if or((H_raw(j,:) == H_unique(i,:)), (H_raw(j,:) == -H_unique(i,:)))
            if or((H_unique(i,:) == H_raw(j,:)), (H_unique(i,:) == -H_raw(j,:)))
                if flag_index == 0
                    flag_index = 1;
                    index_ineq{end+1,1} = ineq_raw_list(j);
                else
                    index_ineq{i,1} = [index_ineq{i,1}; ineq_raw_list(j)];
                end            
            end
        end
    end
    ineq = [ineq index_ineq];
%     ineq{:,end+1} = index_ineq;
end

