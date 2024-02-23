clear
dt = 0.1;
T_f = 5; %[s]
t_acc = 1;%[s]
l = 1;%[m]

V_m = l/(T_f - t_acc);

t = 0 : dt : T_f;
t01 = t(1 : find(t==t_acc));
t12 = t(find(t==t_acc) : find(t==T_f - t_acc));
t23 = t(find(t==T_f - t_acc) : end);

v1 = (V_m / t_acc) * t;
v2 = V_m;
v3 = V_m - (V_m / t_acc) * (t - T_f + t_acc);

plot(t, v1);
hold on
plot(t, v2);
plot(t, v3);

hold off
