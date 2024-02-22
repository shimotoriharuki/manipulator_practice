syms th0 th1 l0 l1 P

%順運動学
fx = l0*cos(th0) + l1*cos(th0+th1);
fy = l0*sin(th0) + l1*sin(th0+th1);
P = [fx; fy];

%ヤコビアン
J = [diff(fx, th0), diff(fx, th1); 
     diff(fy, th0), diff(fy, th1)];

%逆ヤコビアン
J_inv = inv(J);