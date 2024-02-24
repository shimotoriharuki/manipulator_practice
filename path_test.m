clear
% syms t T_f t_acc l V_m
syms t 
T_f = 5;
t_acc = 1;
l = 1;


%速度
V_m = l / (T_f - t_acc);

x1(t) = 1/2 * (V_m / t_acc) * t^2;
x2(t) = 1/2 * t_acc * V_m + (t - t_acc) * V_m;
x3(t) = V_m * (T_f - t_acc) - 1/2 * (V_m / t_acc) * (T_f - t)^2;

v1(t) = diff(x1, t);
v2(t) = diff(x2, t);
v3(t) = (diff(x3, t));

xx1 = int(v1);
xx2 = int(v2);
xx3 = int(v3);


fplot(t, xx1, [0, t_acc]);
hold on
fplot(t, xx2, [t_acc, T_f - t_acc]);
fplot(t, xx3, [T_f - t_acc, T_f]);
hold off