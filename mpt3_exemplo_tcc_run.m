close all

Nsim = 100;
x0 = [-0.5 0 1 -0.1]';
Y = []; U = []; X = [];
model.initialize(x0);
for i=1:Nsim
%    if i==10
%       x0 = x0 + [-1.6; 0.9];
%    end
   %u = expmpc.evaluate(x0);
   u = expmpc.evaluate(x0);
   [x0, y] = model.update(u);
   X = [X; x0'];
   U = [U; u];
   Y = [Y; y];
end
plot(X(:,1))
hold on
plot(X(:,2))
title('Estados')
figure
plot(Y)
title('saida')
figure
plot(U)
title('control')