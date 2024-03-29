clear
clf

%%% 立式 %%%
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

%%% パラメータを代入 %%%
dt = 0.01;
T_f_ = 1; %[s]
t_acc_ = 0.2;%[s]

tt = 0 : dt : T_f_;
x_0 = [0; 0]; %初期位置

x_f = [sqrt(3)*tt; tt]; %目標位置
% x_f = [tt; sin((2*pi) * tt)]; %目標位置

delta_x = diff(x_f(1, :));
delta_y = diff(x_f(2, :));
delta_l = sqrt(power(delta_x, 2) + power(delta_y, 2));
l_ = sum(delta_l);  %[m] %初期位置から目標位置までの軌道の道のりの距離
% l_ = 2*pi;  %[m] %初期位置から目標位置までの軌道の道のりの距離
% l_ = sqrt(2);  %[m] %初期位置から目標位置までの軌道の道のりの距離

fv = subs(fv, T_f, T_f_);
fv = subs(fv, t_acc, t_acc_);
fv = subs(fv, l, l_);

fx = subs(fx, T_f, T_f_);
fx = subs(fx, t_acc, t_acc_);
fx = subs(fx, l, l_);

%プロット
m = 4; %プロットの行
n = 1; %プロットの列

subplot(m, n, 1)
scatter(x_0(1, :) + x_f(1, :), x_0(2, :) + x_f(2, :));
title("軌道")
xlabel("x [m]")
ylabel("y [m]")

subplot(m, n, 2)
fplot(t, fv);
title("v-tグラフ")
xlabel("t [s]")
ylabel("v [m/s]")

subplot(m, n, 3)
fplot(t, fx);
title("x-tグラフ")
xlabel("t [s]")
ylabel("x [m]")

%軌道計算
distance = 0;
delta_path = zeros(2, length(x_f) - 1);
V = l_ / T_f_;
for i = 1 : length(x_f) - 1

    delta_path(:, i) = ([delta_x(i); delta_y(i)] / delta_l(i)) * fv(tt(i)) * dt; %台形速度
    % delta_path(:, i) = ([delta_x(i); delta_y(i)] / delta_l(i)) * V * dt; %一定速度

    distance = distance + delta_l(i);
end

path = [cumsum(delta_path(1, :)); cumsum(delta_path(2, :))];

subplot(m, n, 4)
scatter(path(1, :), path(2, :))
