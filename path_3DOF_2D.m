clear 

%台形速度
syms t_acc T_d  t v1(t) v2(t) v3(t) x1(t) x2(t) x3(t) n

%パス
syms x0 y0 xf yf 

V = n / (T_d - t_acc);

v1(t) = (V / t_acc) * t;
v2(t) = v1(t_acc) + V;
v3(t) = v2(T_d - t_acc) - (V / t_acc) * (t);


% x1(t) = int(v1(t));
% x2(t) = int(v2(t));
% x3(t) = int(v3(t));

tex_x1 = latex(v1(t));
tex_x2 = latex(v2(t));
tex_x3 = latex(v3(t));

%プロット
t_acc_ = 1;
T_d_ = 5;
% x0_ = 0;
% y0_ = 0;
% xf_ = 1;
% yf_ = 1;
n_ = 1;

v1(t) = subs(v1(t), t_acc, t_acc_);
v1(t) = subs(v1(t), T_d, T_d_);
% v1(t) = subs(v1(t), x0, x0_);
% v1(t) = subs(v1(t), y0, y0_);
% v1(t) = subs(v1(t), xf, xf_);
% v1(t) = subs(v1(t), yf, yf_);
v1(t) = subs(v1(t), n, n_);

v2(t) = subs(v2(t), t_acc, t_acc_);
v2(t) = subs(v2(t), T_d, T_d_);
% v2(t) = subs(v2(t), x0, x0_);
% v2(t) = subs(v2(t), y0, y0_);
% v2(t) = subs(v2(t), xf, xf_);
% v2(t) = subs(v2(t), yf, yf_);
v2(t) = subs(v2(t), n, n_);


v3(t) = subs(v3(t), t_acc, t_acc_);
v3(t) = subs(v3(t), T_d, T_d_);
% v3(t) = subs(v3(t), x0, x0_);
% v3(t) = subs(v3(t), y0, y0_);
% v3(t) = subs(v3(t), xf, xf_);
% v3(t) = subs(v3(t), yf, yf_);
v3(t) = subs(v3(t), n, n_);


fplot(v1(t), [0, t_acc_]);
hold on
fplot(v2(t), [t_acc_, T_d_ - t_acc_]);
fplot(v3(t), [T_d_ - t_acc_, T_d_]);
hold off