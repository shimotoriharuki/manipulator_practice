clear
syms th0 th1 th2 l0 l1 l2 l3 l4

%順運動学
f_x = cos(th0) * (l2 * cos(th1) + l3 * cos(th1 + th2));
f_y = sin(th0) * (l2 * cos(th1) + l3 * cos(th1 + th2));
f_z = l0 + l1 + l2 * sin(th1) + l3 * sin(th1 + th2) - l4;
P = [f_x; f_y; f_z];

%ヤコビアン
J = [diff(f_x, th0), diff(f_x, th1), diff(f_x, th2);
     diff(f_y, th0), diff(f_y, th1), diff(f_y, th2);
     diff(f_z, th0), diff(f_z, th1), diff(f_z, th2)];

%逆ヤコビアン
J_inv = inv(J);