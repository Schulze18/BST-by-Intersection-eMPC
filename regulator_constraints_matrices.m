function [G, W, E, S] = regulator_constraints_matrices(B, Clinha, Su, Sx, H, F, Nu, Ny, Umax, Umin, Xmax, Xmin, Ymax, Ymin)
%[G, W, E, S] = constraints_matrices_reformulation(C, Sx, H, F, Nu, Umax, Umin, Xmax, Xmin)
%
%Return the constraints matrices G, W and E from the reformulated cost function with U to
%be opmitze.
%Inputps:
%        b - matrices from the state-space equation x[t+1] = A*x[t] + B*u[t]
%
%        H, F - from the cost function
%
%        Nu - control horizon
%
%        Umax, Umin - contraints on control action
%
%        Xmax, Xmin - constraints on state value
%
%        Ymax, Ymin - constraints on state value
%
%Outpus:
%        G, W, E and S - from the matrix inequalities G*z <= W + S*x(t) and GU <= W + Ex(t) 
%
%Algoritm based on the paper "The explicit linear quadratic regulator for
%constrained systems" by A. Bemporad, M. Morari, V. Dua, and E. Pistikopoulos. 

    G_u = [];
    G_x = [];
    G_y = [];
    W_u = [];
    W_x = [];
    W_y = [];
    E_u = [];
    E_x = [];
    E_y = [];
    S = [];
    
    num_state = size(B,1);        %Number of elements in the state vector
    num_control = size(B,2);       %Number of control inputs 
    num_out = 1;
    %U constraints
    if isempty(Umax) == 0
        E_u = zeros(2*num_control*Nu,num_state);        
        W_u(1:num_control,1) = Umax;
        %W_u((1+num_control):(2*num_control),1) = -Umin;
        G_u(1:num_control,1) = ones(size(Umax,2),1);
        %G_u((1+num_control):(2*num_control),1) = -ones(size(Umin,2),1);  
        for i = 1:(Nu-1)
            G_u = blkdiag(G_u, ones(size(Umax,2),1));
            W_u = [W_u; Umax];
        end
        G_u = [G_u; -G_u];
        W_u = [W_u; W_u];
    end
    
    %X constraints
    if isempty(Xmax) == 0
        G_x = zeros(2*num_state,Nu);
        E_x = -eye(num_state);
        E_x = [E_x; eye(num_state)];
        W_x = Xmax';
        W_x = [W_x; (-Xmin)'];
    end
  
    %Y constraints
    if isempty(Ymax) == 0
        W_y(1:num_out,1) = Ymax;
%         G_y(1:num_out,1) = ones(size(Ymax,2),1);
        
        for i = 1:Ny
            W_y = [W_y; Ymax];
        end
        G_y = [Clinha*Su; -Clinha*Su];
        W_y = [W_y; W_y];
        E_y = [-Clinha*Sx; Clinha*Sx];
    end
    
%     G_u
%     G_y
%     E_u
%     E_y
    W = [W_u; W_x; W_y];
    G = [G_u ; G_x; G_y];
    E = [E_u ; E_x; E_y];
 
    S = E + G*inv(H)*F';
end
