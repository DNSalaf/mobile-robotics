function [x1,x2,y1,y2,z1,z2,time,norm_erro_vec,xf_erro_vec, yf_erro_vec,zf_erro_vec,rho_f_erro_vec,a_f_erro_vec,b_f_erro_vec,veloc_lin_vec1,veloc_lin_vec2] = task2(animado)
%TASK 2
%Formação com dois robôs aéreos, do tipo quadrimotor.

%inicialização das variáveis
x1 = 0;
y1 = 0;
z1 = 0.75;
psi1 = 0;

x2 = -2;
y2 = 0;
z2 = 0.75;
psi2 = 0;

xf = 2;
yf = 1;
zf = 3;
pf = 2;
af = 0;
bf = 0;

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

Ts = 200e-3; %periodo de amostragem dado

v1 = 10*[1 1 1 1 1 1];
v2 = 10*[2 2 2 2 2 2];
L1 = diag(v1);
L2 = diag(v2);

q_des = [xf yf zf pf af bf]'; 
v = zeros(1,8);	

q_des = [xf yf zf pf af bf]';   		%não varia no tempo, pois é tarefa de posicionamento
q_erro = q_des; %inicialização do erro
q_erro_stop = [10 10 10];
counter = 0;

norm_erro_vec = [0 0 0];
xf_erro_vec = [0 0 0];
yf_erro_vec = [0 0 0];
zf_erro_vec = [0 0 0];
rho_f_erro_vec = [0 0 0];
a_f_erro_vec = [0 0 0];
b_f_erro_vec = [0 0 0];

veloc_lin_vec1 = zeros(1,4);
veloc_lin_vec2 = zeros(1,4);

while norm(abs(q_erro)) > 0.1 %mudar esse criterio dps pra igual o da task1
	q = [x1(counter+1) y1(counter+1) z1(counter+1) sqrt((x2(counter+1)-x1(counter+1))^2 + (y2(counter+1)-y1(counter+1))^2 + (z2(counter+1)-z1(counter+1))^2) atan((z2(counter+1)-z1(counter+1))/sqrt((x2(counter+1)-x1(counter+1))^2 + (y2(counter+1)-y1(counter+1))^2)) atan((y2(counter+1)-y1(counter+1))/(x2(counter+1)-x1(counter+1)))]';
    q_erro = q_des - q; %deve decair para zero, mas nunca chegar a zero
    q_ref = L1*tanh(inv(L2)*q_erro);
   
    rho_f = q(4);
    a_f = q(5);
    b_f = q(6);
   
	J_inv = [1 0 0 0 0 0;
             0 1 0 0 0 0;
             0 0 1 0 0 0;
             1 0 0 cos(af)*cos(bf) -pf*sin(af)*cos(bf) -pf*cos(af)*sin(bf);
             0 1 0 sin(af)*cos(bf) pf*cos(af)*cos(bf) -pf*sin(af)*sin(bf);
             0 0 1 sin(bf) 0 pf*cos(bf)]; %jacobiano
    
	x_ref = J_inv*q_ref;
    x_ref = [x_ref(1:3)' 0 x_ref(4:6)' 0]';
    
    K_inv = [cos(psi1) sin(psi1) 0 0 0 0 0 0;
            -sin(psi1) cos(psi1) 0 0 0 0 0 0;
             0 0 1 0 0 0 0 0;
             0 0 0 1 0 0 0 0;
             0 0 0 0 cos(psi2) sin(psi2) 0 0;
             0 0 0 0 -sin(psi2) cos(psi2) 0 0;
             0 0 0 0 0 0 1 0;
             0 0 0 0 0 0 0 1]; 
    
    v_ref = K_inv*x_ref
	v1 = v_ref(1:4);
	v2 = v_ref(5:8);
   
    M = eye(4); %M é função de psi1 e psi2, porém estes serão sempre zero e portanto gerarão M igual a identidade 4x4
    
    h1 = M*v1;
    h2 = M*v2;
     
	x1(counter+2) = h1(1)*Ts + x1(counter+1);
	y1(counter+2) = h1(2)*Ts + y1(counter+1);
	z1(counter+2) = h1(3)*Ts + z1(counter+1);
	x2(counter+2) = h2(1)*Ts + x2(counter+1);
	y2(counter+2) = h2(2)*Ts + y2(counter+1);
	z2(counter+2) = h2(3)*Ts + z2(counter+1);
  
    if(animado)
        plot3(x1(counter+1),y1(counter+1),z1(counter+1),'b--*');
        xlabel('x[m]');
        ylabel('y[m]');
        zlabel('Z[m]');
        grid on
        hold on
        plot3(x1,y1,z1,'b');
        axis([-3 6 0 3 0 6]);  
        plot3(x2(counter+1),y2(counter+1),z2(counter+1),'r--*');
        plot3(x2,y2,z2,'r');
        pause(Ts)
        hold off
    end

    v = [v1' v1'];
       
    counter = counter + 1; %contador de iterações
    
     norm_erro_vec(counter) = abs(norm(q_erro));
     xf_erro_vec(counter) = q_erro(1);
     yf_erro_vec(counter) = q_erro(2);
     zf_erro_vec(counter) = q_erro(3);
     rho_f_erro_vec(counter) = q_erro(4);
     a_f_erro_vec(counter) = q_erro(5);
     b_f_erro_vec(counter) = q_erro(6);
          
     veloc_lin_vec1(counter) = v1(1);
     veloc_lin_vec2(counter) = v2(1);
             
end
%calculando tempo de percurso
tempo_total = counter*Ts;
time = linspace(0,tempo_total,counter);
end