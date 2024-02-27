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
T_f_ = 1; %[s]
t_acc_ = 0.2;%[s]

tt = 0 : dt : T_f_;
x_0 = [0; 0]; %初期位置
% x_f = [1; 0]; %目標位置
x_f = [tt; sin((pi/2) * tt)]; %目標位置
l_ = 2*pi/4;%[m] %初期位置から目標位置までの軌道の道のりの距離
% l_ = [x_f(1, end) - x_0(1); x_f(2, end) - x_0(2)];%[m] %初期位置から目標位置までの軌道の道のりの距離

fv = subs(fv, T_f, T_f_);
fv = subs(fv, t_acc, t_acc_);
fv = subs(fv, l, l_);
% fv_x = subs(fv, l, l_(1)); %x成分
% fv_y = subs(fv, l, l_(2)); %y成分

fx = subs(fx, T_f, T_f_);
fx = subs(fx, t_acc, t_acc_);
fx = subs(fx, l, l_);
% fx_x = subs(fx, l, l_(1)); %x成分
% fx_y = subs(fx, l, l_(2)); %y成分

%プロット
m = 4; %プロットの行
n = 2; %プロットの列

% subplot(m, n, [1, 2])
scatter(x_0(1, :) + x_f(1, :), x_0(2, :) + x_f(2, :));
% title("軌道")
% xlabel("x [m]")
% ylabel("y [m]")
axis equal

% subplot(m, n, 3)
% fplot(t, fv_x);
% title("vx-tグラフ")
% xlabel("t [s]")
% ylabel("vx [m/s]")
% 
% subplot(m, n, 4)
% fplot(t, fv_y);
% title("vy-tグラフ")
% xlabel("t [s]")
% ylabel("vy [m/s]")
% 
% subplot(m, n, 5)
% fplot(t, fx_x);
% title("x-tグラフ")
% xlabel("t [s]")
% ylabel("x [m]")
% 
% subplot(m, n, 6)
% fplot(t, fx_y);
% title("y-tグラフ")
% xlabel("t [s]")
% ylabel("y [m]")

% subplot(m, n, [7, 8])
n = T_f_ / dt;
distance = 0;
ddx = zeros(2, length(x_f) - 1);
% dit = zeros(2, length(x_f) - 1);
for i = 1 : length(x_f) - 1
    delta_x = x_f(1, i + 1) - x_f(1, i);
    delta_y = x_f(2, i + 1) - x_f(2, i);
    d = sqrt(delta_x^2 + delta_y^2);
    V = d / T_f_;

    ddx(:, i) = ([delta_x; delta_y] / d) * fv(tt(i)) * dt;
    distance = distance + d;
    

end

dit = [cumsum(ddx(1, :)); cumsum(ddx(2, :))];
% d_path_x = ((x_f(1, :) - x_0(1)) / l_(1)) .* fv_x(tt) * dt;
% d_path_y = ((x_f(2, :) - x_0(2)) / l_(2)) .* fv_y(tt) * dt;
% path_x = x_0(1) + [cumsum(d_path_x)];
% path_y = x_0(2) + [cumsum(d_path_y)];

% d_path = ((x_f - x_0) / l_) .* fv(tt) * dt;
% path = x_0 + [cumsum(d_path(1, :)); cumsum(d_path(2, :))];
% path_x = x_0(1, :) + (x_f(1, end) - x_0(1, :) / l_(1, :)) .* fx_x(tt);
% path_y = x_0(2, :) + (x_f(2, :) - x_0(2, :) / l_(2, :)) .* fx_y(tt);

% scatter(path_x, path_y);
% title("手先位置")
% xlabel("x [m]")
% ylabel("y [m]")
% axis equal