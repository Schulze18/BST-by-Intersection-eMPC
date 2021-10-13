function [ineq] = verifiy_total_ineq(Regions)
%[ineq] = verifiy_total_ineq(Regions)
%List all inequations in a set of Regions
%Inputs:
%       Regions - cell array with all the regions from the explicit
%       controller  
%
%Outputs:
%       ineq - set of all the inequations
%   
    total_ineq = 0;
    ineq = {};
    for i = 1:size(Regions,1)
        Aj = Regions{i,1};
        bj = Regions{i,2}; 
        for j = 1:size(Aj,1)           
            flag = 0;
            total_ineq = total_ineq + 1;
            
            %
            for k = 1:size(ineq,1)
                test_matrix = [Aj(j,:) bj(j,:); ineq{k,1} ineq{k,2}]; 
                rank_test = rank(test_matrix);
                if rank_test ~= 2
                    test_matrix;
                    flag = 1;
                    ineq{k,3} = [ineq{k,3};i];
                else
                    test_matrix;
                end
            end
            
            if flag == 0
                ineq{end+1,1} = Aj(j,:);
                ineq{end,2} = bj(j,:);
                ineq{end,3} = i;
                
            end
            
        end
    end
end

