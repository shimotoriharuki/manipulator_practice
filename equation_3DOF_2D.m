syms th0 th1 th2 l0 l1 P

%順運動学
f_x = l0*cos(th0) + l1*cos(th0+th1);
f_y = l0*sin(th0) + l1*sin(th0+th1);
f_th = th2;
P = [f_x; f_y; f_th];

%ヤコビアン
J = [diff(f_x, th0), diff(f_x, th1), diff(f_x, th2);
     diff(f_y, th0), diff(f_y, th1), diff(f_y, th2);
     diff(f_th, th0), diff(f_th, th1), diff(f_th, th2)];

%逆ヤコビアン
J_inv = inv(J);