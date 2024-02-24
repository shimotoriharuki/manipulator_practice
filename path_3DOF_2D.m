syms T_f t_acc l v1(t) v2(t) v3(t) V_m

v1(t) = (V_m / t_acc) * t;
v2(t) = v1(t_acc);
v3(t) = v2(T_f - t_acc) - (V_m / t_acc) * (t - T_f + t_acc);

fv = piecewise((0 <= t) & (t <= t_acc), v1, (t_acc <= t) & (t <= T_f - t_acc), v2, (T_f - t_acc <= t) & (t <= T_f), v3); % まとめる

%位置
x1 = int(v1);
x2 = int(v2);
x3 = int(v3);

%積分定数
x1 = x1 + 0;
x2 = x2 + (x1(t_acc) - x2(t_acc));
x3 = x3 + (x2(T_f - t_acc) - x3(T_f - t_acc));

fx = piecewise((0 <= t) & (t <= t_acc), x1, (t_acc <= t) & (t <= T_f - t_acc), x2, (T_f - t_acc <= t) & (t <= T_f), x3); % まとめる


%tex形式で出力
fv_latex = latex(fv);
fx_latex = latex(fx);

%最大速度
% V_m = l / (T_f - t_acc);
fx = subs(fx, V_m, l / (T_f - t_acc));
fv = subs(fv, V_m, l / (T_f - t_acc));

%数値を代入してプロット
T_f_ = 5; %[s]
t_acc_ = 2;%[s]
l_ = 1;%[m]

fv = subs(fv, T_f, T_f_);
fv = subs(fv, t_acc, t_acc_);
fv = subs(fv, l, l_);

fx = subs(fx, T_f, T_f_);
fx = subs(fx, t_acc, t_acc_);
fx = subs(fx, l, l_);

figure(1)
subplot(3, 1, 1)
fplot(t, fv);
title("v-tグラフ")
xlabel("t [s]")
ylabel("v [m/s]")

subplot(3, 1, 2)
fplot(t, fx);
title("x-tグラフ")
xlabel("t [s]")
ylabel("x [m]")

subplot(3, 1, 3)
tt = linspace(0, T_f_, 100);
scatter(fx(tt), 1);
title("手先位置")
xlabel("x [m]")
ylabel("y [m]")
axis equal