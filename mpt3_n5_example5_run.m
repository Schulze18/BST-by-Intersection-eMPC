Nsim = 40;
x0 = [0 0.2 0.3 -0.1]';
Y = []; U = [];
model.initialize(x0);
for i=1:Nsim
%    if i==10
%       x0 = x0 + [-1.6; 0.9];
%    end
   %u = expmpc.evaluate(x0);
   u = expmpc.evaluate(x0, 'y.reference', 1);
   [x0, y] = model.update(u);
   U = [U; u];
   Y = [Y; y];
end
plot(Y)
title('saida')
figure
plot(U)
title('control')