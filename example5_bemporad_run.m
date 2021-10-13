% clc
close all

Nsim = 100;
x = zeros(Nstate,Nsim);
y = zeros(Nsim,Nout);
% x(:,1) = [-0.5 0 1 -0.1]';
x(:,1) = [0.2 0.1 0.1 -0.1]';
for i = 1:Nsim
   
    for j = 1:size(Regions,1)
        A_CRi = Regions{j,1};
        b_CRi = Regions{j,2};
        flag = 0;
        for k = 1:size(A_CRi,1)
            if(A_CRi(k,:)*x(:,i) > b_CRi(k))
                flag = 1;
            end
        end
        if flag == 0
            index = j;
        end
    end
    u(i) = Regions{index,3}*x(:,i) + Regions{index,4};
%     A*x(:,i)
%     B*u
    x(:,i+1) = Ad*x(:,i)+Bd*u(i);%+0.02*[rand(1) ; rand(1) ];
    y(i) = Cd*x(:,i);
    index_vec(i) = index;
    index = 0;

end


figure(10)
stairs(x(1,:))
hold on
stairs(x(2,:),'r')
stairs(x(3,:),'b')
stairs(x(4,:),'g')
legend('x1','x2','x3','x4')

figure(11)
stairs(u)
legend('u')
title('u')

figure(12)
plot(index_vec)
title('Index')

figure(13)
plot(y)
title('Saida y')