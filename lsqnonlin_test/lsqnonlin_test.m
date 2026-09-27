t = -100:.1:100;

fun = @(x)(x(1)*c(2)-20).^2/4;
% plot(t, fun(t,c))

lsqnonlin(fun, [10 1], [0 0], [20 2])

