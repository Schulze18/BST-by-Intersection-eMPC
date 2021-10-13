close all

Nsim = 20;
x0 = [10 -5.0 1.5]';
Y = []; U = []; X = [];
model.initialize(x0)%,'y.reference',x0(1));
% model.y.reference = x0(1);
% model.initialize('y.reference',x0(1));
for i=1:Nsim
%    if i==10
%       x0 = x0 + [-1.6; 0.9];
%    end
   u = expmpc.evaluate(x0);
%    u = expmpc.evaluate(x0,'y.reference',x0(1));
   [x0, y] = model.update(u);
   X = [X x0];
   U = [U u];
   Y = [Y y];
end
%%

plot(X(1,:))
hold on
plot(X(2,:))
plot(X(3,:))
title('Estados')
grid on

figure
plot(Y(1,:))
hold on
plot(Y(2,:))
plot(Y(3,:))
title('saida')
grid on
%%
figure
plot(U(1,:))
hold on
grid on
plot(U(2,:))
title('control')
grid on