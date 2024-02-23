clear
dt = 0.1;
T_f = 5; %[s]
t_acc = 1;%[s]
l = 1;%[m]

syms v1(t) v2(t) v3(t)

V_m = l / (T_f - t_acc);

v1(t) = (V_m / t_acc) * t;
v2(t) = V_m;
v3(t) = V_m - (V_m / t_acc) * (t - T_f + t_acc);

fplot(t, v1, [0, t_acc]);
hold on
fplot(t, v2, [t_acc, T_f - t_acc]);
fplot(t, v3, [T_f - t_acc, T_f]);

hold off
