clear
dt = 0.1;
T_f = 5; %[s]
t_acc = 1;%[s]
l = 1;%[m]

syms v1(t) v2(t) v3(t)

%速度
V_m =l / (T_f - t_acc);

v1(t) = (V_m / t_acc) * t;
v2(t) = v1(t_acc);
v3(t) = v2(T_f - t_acc) - (V_m / t_acc) * (t - T_f + t_acc);


subplot(2, 1, 1)
fplot(t, v1, [0, t_acc]);
hold on
fplot(t, v2, [t_acc, T_f - t_acc]);
fplot(t, v3, [T_f - t_acc, T_f]);
hold off

%位置
x1 = int(v1);
x2 = int(v2);
x3 = int(v3);

%積分定数
x2 = x2 + (x1(t_acc) - x2(t_acc));
x3 = x3 + (x2(T_f - t_acc) - x3(T_f - t_acc));

subplot(2, 1, 2)
fplot(t, x1, [0, t_acc]);
hold on
fplot(t, x2, [t_acc, T_f - t_acc]);
fplot(t, x3, [T_f - t_acc, T_f]);
hold off
% xlim([0, 5])
% ylim([0, 1])