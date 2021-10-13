function [xc , r, diagnostics] = point_in_region_chebyshev(A, b, Nstate, options)
%[xc , r, diagnostics] = point_in_region_chebyshev(A, b, options)
%
%Return the center of the largest possible ball that can be placed inside
%the region defined by Ax<=b (Chebyshev center) and that Vz(x) is feasible.
%Inputs:
%       A, b - matrices that define the polyhedral Ax <= b
%       
%       Nstate, Ncontrol, Nout - number of states, control actions and
%       outputs of the system
%
%       Ny, Nu - prediction and control horizon 
%
%       options - solver options for yalmip 
%
%Outputs:
%       xc - Chebyshev center
%
%       r - radius of the Chebyshev ball
%
%       diagnostics - returns the "optimize" function status
%
%Algoritm based on the paper "The explicit linear quadratic regulator for
%constrained systems" by A. Bemporad, M. Morari, V. Dua, and E. Pistikopoulos. 
   
    
    xc = sdpvar(Nstate,1,'full');
    r = sdpvar(1);
    
    LMI = [];
    for i = 1:size(A,1)
        LMI = [LMI; A(i,:)*xc + r*sqrt(sum(A(i,:).^2)) <= b(i)];
    end

    
    diagnostics = optimize(LMI,-r,options);
%     check(LMI)
    
    xc = double(xc);
    r = double(r);
end

