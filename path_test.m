clear
dt = 0.1;
T_f = 5; %[s]
t_acc = 1;%[s]
l = 1;%[m]

syms t

%速度
V_m = l / (T_f - t_acc);

x1(t) = 1/2 * (V_m / t_acc) * t^2;
x2(t) = 1/2 * t_acc * V_m + (t - t_acc) * V_m;
x3(t) = V_m * (T_f - t_acc) - 1/2 * (V_m / t_acc) * (T_f - t)^2;

fplot(t, x1, [0, t_acc]);
hold on
fplot(t, x2, [t_acc, T_f - t_acc]);
fplot(t, x3, [T_f - t_acc, T_f]);
hold off