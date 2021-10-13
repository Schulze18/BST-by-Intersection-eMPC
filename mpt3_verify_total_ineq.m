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
    if false
        H = cat(1, optimizer_mpt3.Set.H);

        % find unique "a" from a'*x<=0 and a'*x>=0
        H = round(H*10^r_coef)/10^r_coef;
        nH_orig = size(H, 1);
        H = unique(H, 'rows');
    %     H = uniquetol(H, 'ByRows',1e-4);
        [i, j] = ismember(H, -H, 'rows');
        idx1 = find(i==0);
        idx2 = j(j > (1:length(j))');
        H = H(unique([idx1; idx2]), :);
        ineq = {};
        for k = 1:size(H,1)
            ineq{k,1} = H(k,1:(end-1));
            ineq{k,2} = H(k,end);
        end
    end
    %%
    ineq_raw_list = [];
    raw_H = cat(1, optimizer_mpt3.Set.H);
    for k = 1:size(optimizer_mpt3.Set,1)
        num_ineq = size(optimizer_mpt3.Set(k).A,1);
        ineq_raw_list = [ineq_raw_list; ones(num_ineq,1)*k];
    end
% %     index_ineq = {};
% %     for i = 1:size(H,1)
% %         flag_index = 0;
% %         for j = 1:size(index_ineq_raw,1)
% %             if or((raw_H(j,:) == H(i,:)), (raw_H(j,:) == -H(i,:)))
% % %                 if isempty(index_ineq)
% % %                     index_ineq = index_ineq_raw(j);
% % % %                 elseif isempty(index_ineq{i})
% % %                 elseif size(index_ineq,1) < i
% % %                     index_ineq{i} = index_ineq_raw(j);
% % %                 else
% % %                     index_ineq{i} = [index_ineq{i}; index_ineq_raw(j)];
% % %                 end
% %                 if flag_index == 0
% %                     flag_index = 1;
% %                     index_ineq{end+1} = index_ineq_raw(j);
% %                 else
% %                     index_ineq{i} = [index_ineq{i}; index_ineq_raw(j)];
% %                 end            
% %             end
% %         end
% %     end
% %     ineq = {ineq index_ineq};
% 
%% %%%%%%%%%%%

    H = round(raw_H*10^r_coef)/10^r_coef;
    [H, index_unique] = unique(H, 'rows');
    new_region_index = ineq_raw_list(index_unique);
%     H = [
    [i, j] = ismember(H, -H, 'rows');
    idx1 = find(i==0);
    idx2 = j(j > (1:length(j))');
    H = H(unique([idx1; idx2]), :);
    final_index =  new_region_index(unique([idx1; idx2]));
    ineq = {};
    for k = 1:size(H,1)
        ineq{k,1} = H(k,1:(end-1));
        ineq{k,2} = H(k,end);
    end
    
end

