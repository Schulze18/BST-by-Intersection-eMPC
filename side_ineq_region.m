function [result, diagnostics1, diagnostics2] = side_ineq_region(ineq, Region, options)
%[result, diagnostics1, diagnostics2] = side_ineq_region(ineq, Region, options)
%
%Find if the "Region" is on the right or in the left of the "ineq"
%Inputs:
%       ineq - inequation definition
%
%       Region - region to be testes
%
%       options - solver options for yalmip  
%
%Outputs:
%       result - indicate the side: 0 - no answer, 1 - left, 2 - right,
%       3 - both sides
%
%       diagnostics1 - diagnostics for the "left" test
%
%       diagnostics2 - diagnostics for the "right" test
    
    tol = 1e-6; 

    %Test if ineq is in Region definition
    result = verify_ineq_in_region(Region, ineq);
% % %     A_region = Region{1,1};
% % %     b_region = Region{1,2};
% % %     
% % %     A_ineq = ineq{1,1};
% % %     b_ineq = ineq{1,2};
% % %     
% % %     for i = 1:size(A_region,1)
% % %         test_matrix = [A_region(i,:) b_region(i,:); A_ineq b_ineq]; 
% % %         rank_test = rank(test_matrix);
% % %                 
% % %         sign_test = sum(test_matrix(1,:))/sum(test_matrix(2,:));
% % %     
% % %         if rank_test == 1 && sign_test > 0
% % %             result = 1;
% % %             break
% % %         elseif rank_test == 1 && sign_test < 0
% % %             result = 2;
% % %             break
% % %         end
% % %    
% % %     end
  
    %Only if it is not from the Region Definition
    if result == 0
    
        A_ineq = ineq{1,1};
        b_ineq = ineq{1,2};
       
        %Check <= sign
        x = sdpvar(size(A_ineq,2),1,'full');

        objective = A_ineq*x - b_ineq;
% % % % %         options = sdpsettings;
% % % % % 
% % % % %         options.solver='sedumi';
% % % % % %          options.solver='sdpt3';
% % % % %         options.verbose = 0;
% % % % %         options.cachesolvers = 1;
% % % % % 
% % % % % %         options.sdpt3.maxit = 20;
% % % % % %         options.sdpt3.steptol = 1.0000e-08;
% % % % % %         options.sdpt3.gaptol = 5.000e-8;
% % % % %         
% % % % %         options.sedumi.eps = 5.0000e-04;
% % % % %         options.sedumi.maxiter = 20;


        A_region = Region{1,1};
        b_region = Region{1,2};

        LMI1 = [];
        for i = 1:size(A_region,1)
            LMI1 = [LMI1; A_region(i,:)*x <= (b_region(i,:) - tol)];
        end
%          LMI1 = [LMI1;  A_ineq*x <= b_ineq];
        
        diagnostics1 = optimize(LMI1,objective,options);

        %if double(objective) <= -tol || isequal( diagnostics1.info,'Unbounded objective function (SeDuMi-1.3)')
%         if (sum(ismember('Unbounded objective function', diagnostics1.info)) == length('Unbounded objective function'))
%            disp('stop') 
%         end
        
        
        %if (double(objective) <= -tol || (sum(ismember('Unbounded objective function', diagnostics1.info)) == length('Unbounded objective function')))
        if (double(objective) <= 0 || (sum(ismember('Unbounded objective function', diagnostics1.info)) == length('Unbounded objective function')))
            result = result + 1; 
        end

        %Check >= sign
        x2 = sdpvar(size(A_ineq,2),1,'full');
        objective2 = -(A_ineq*x2 - b_ineq);
        LMI2 = [];
        for i = 1:size(A_region,1)
            LMI2 = [LMI2; A_region(i,:)*x2 <= (b_region(i,:) - tol)];
        end
%          LMI1 = [LMI2; A_ineq*x2 >= b_ineq];
        
        diagnostics2 = optimize(LMI2,objective2,options);

        
%         if (sum(ismember('Unbounded objective function', diagnostics2.info)) == length('Unbounded objective function'))
%            disp('stop') 
%         end
              
        if (double(objective2) <= 0 || (sum(ismember('Unbounded objective function', diagnostics2.info)) == length('Unbounded objective function')))
        %if (double(objective2) <= -tol || (sum(ismember('Unbounded objective function', diagnostics2.info)) == length('Unbounded objective function')))
        %if double(objective2) <= -tol || isequal( diagnostics2.info,'Unbounded objective function (SeDuMi-1.3)')   
            result = result + 2; 
        end
        
        %%% Debug
%         if (isequal( diagnostics1.info,'Unbounded objective function (SeDuMi-1.3)') && (double(objective) > -tol))
%             disp('unbounded 1')
%         end
%         
%         if (isequal( diagnostics2.info,'Unbounded objective function (SeDuMi-1.3)') && (double(objective2) > -tol))
%             disp('unbounded 2')
%         end
        if result == 0
            disp('deu ruim')
        end
    
        
        yalmip('clear');
    end
     
end

