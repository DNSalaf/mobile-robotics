function task2()
%TASK 2
%Formação com dois robôs aéreos, do tipo quadrimotor.

x1 = 0;
y1 = 0;
z1 = 0.75;
psi1 = 0;

x2 = -2;
y2 = 1;
z2 = 0.75;
psi2 = 0;

xf = 2;
yf = 1;
zf = 3;
rho_f = 2;
a_f = 0;
B_f = 0;

k1 = 0.8417;
k2 = 0.18227;
k3 = 0.8354;
k4 = 0.17095;
k5 = 3.966;
k6 = 4.001;
k7 = 9.8524;
k8 = 4.7295;

Ku = diag([k1 k3 k5 k7]);
Kv = diag([k2 k4 k6 k8]);

Ts = 200e-3;

end