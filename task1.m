function task1()
%TASK 1
%Formação com dois robôs Pioneer 3-DX (uniciclo).

%%%%%%%%%%%%%%%%%%%%% Camada de planejamento e inicialização de variáveis %%%%%%%%%%%%%%%%%%%%%
%variáveis de formação
xf = 2;
yf = 3;
rho_f = 2;
a_f = 0;

x1 = 0;
y1 = 0;
psi1 = 0;
a1 = 1;

x2 = 1;
y2 = -2;
psi2 = 0;
a2 = 1; 

theta = [0.2604 0.2509 -0.000499 0.9965 0.00263 1.0768];

Ts = 100e-3; %O período de amostragem considerado deve ser de 100 ms

v = zeros(1,4);
vd = zeros(1,4);
vd_anterior = zeros(1,4);
vdp = zeros(1,4);

lu = 1; lw = 1;
ku = 0.2; kw = 0.2;
v1 = [120 120 120 120];
v2 = [0.48 0.48 0.48 0.48];
L = diag(v1);
k = diag(v2);

q_des = [xf yf rho_f a_f]'; %não varia no tempo, pois é tarefa de posicionamento

%%%%%%%%%%%%%%%%%%%%% Camada de Controle %%%%%%%%%%%%%%%%%%%%%
q_erro = q_des; %inicialização do erro
counter = 0;
img = figure('Name','Alaf do Nascimento Santos');
while abs(norm(q_erro)) > 0.1 %quanto menor a norma, mais proximo do ponto desejado e maior o tempo de percurso
    q = [x1(counter+1) y1(counter+1) sqrt((x2(counter+1) - x1(counter+1))^2 + (y2(counter+1) - y1(counter+1))^2) atan((y2(counter+1) - y1(counter+1))/(x2(counter+1) - x1(counter+1)))]'; %vai ser a realimentação
    q_erro = q_des - q; %deve decair para zero, mas nunca chegar a zero
    q_ref = L*tanh(inv(L)*k*q_erro);

    rho_f = q(3);
    a_f = q(4);
    J_inv = [1 0 1 0;
             0 1 0 1;
             0 0 cos(a_f) (-rho_f*sin(a_f));
             0 0 sin(a_f) (rho_f*cos(a_f))]; %jacobiano

    x_ref = J_inv*q_ref;

    K_inv = [cos(psi1) sin(psi1) 0 0;
             (-sin(psi1)/a1) (cos(psi1)/a1) 0 0;
             0 0 cos(psi2) sin(psi2);
             0 0 (-sin(psi2)/a2) (cos(psi2)/a2)];

    vr = K_inv*x_ref; %contem as velocidades lineares e angulares
     
    
%%%%%%%%%%%%%%%%%%%%% Camada dos Robôs %%%%%%%%%%%%%%%%%%%%%
%a partir daqui, preciso passar vref1 e vref2 pelo compensador dinâmico,
    
    %------------Calculando a dinamica para cada robô--------------
    vd = vr';
	vdp = (vr' - vd_anterior)/Ts;
	vd_anterior = vd;  
    
    vd1 = [vd(1) vd(2)]'; vdp1 = [vdp(1) vdp(2)]'; v1 = [v(1) v(2)]';
    vd2 = [vd(3) vd(4)]'; vdp2 = [vdp(3) vdp(4)]'; v2 = [v(3) v(4)]';
    
	v_erro1 = vd1 - v1;
	F1 = [theta(4) 0;
          0 (theta(6) + (theta(5) - theta(3))*vd1(1))];
	C1 = [0 -theta(3)*vd1(2);
          theta(3)*vd1(2) 0];
	H1 = [theta(1) 0;
          0 theta(2)];
	T1 = [lu 0;0 lw]*[tanh(ku*v_erro1(1)/lu);tanh(kw*v_erro1(2))];
	
    vr1 = T1 + H1*vdp1 + C1*vd1 + F1*vd1;
    
    
    v_erro2 = vd2 - v2;
	F2 = [theta(4) 0;
          0 (theta(6) + (theta(5) - theta(3))*vd2(1))];
	C2 = [0 -theta(3)*vd2(2);
          theta(3)*vd2(2) 0];
	H2 = [theta(1) 0;
          0 theta(2)];
	T2 = [lu 0;0 lw]*[tanh(ku*v_erro2(1)/lu);tanh(kw*v_erro2(2))];
    
    vr2 = T2 + H2*vdp2 + C2*vd2 + F2*vd2;
     
    u1 = v(1); ur1 = vr1(1); w1 = v(2); wr1 = vr1(2);
    u2 = v(3); ur2 = vr2(1); w2 = v(4); wr2 = vr2(2);  
    
    %---------------Robô 1-----------
    vp1 = [1/theta(1) 0;
           0 1/theta(2)];
   
    vp1 = vp1*[ur1;
              wr1];

    vp1 = vp1 + [theta(3)/theta(1)*w1^2 - theta(4)/theta(1)*u1;
                -theta(5)/theta(2)*u1*w1 - theta(6)/theta(2)*w1];
    %---------------Robô 2-----------
    vp2 = [1/theta(1) 0;
           0 1/theta(2)];
   
    vp2 = vp2*[ur2;
              wr2];

    vp2 = vp2 + [theta(3)/theta(1)*w2^2 - theta(4)/theta(1)*u2;
                -theta(5)/theta(2)*u2*w2 - theta(6)/theta(2)*w2];
    
    %------------Calculando a cinematica para cada robô--------------   
   
    %---------------Robô 1-----------
    v1 = vp1*Ts + v(1:2)';
    u1 = v1(1); w1 = v1(2);
    h1 = [u1*cos(psi1)-a1*w1*sin(psi1);
           u1*sin(psi1)+a1*w1*cos(psi1)];

   %---------------Robô 2-----------
    v2 = vp2*Ts + v(3:4)';
    u2 = v2(1); w2 = v2(2);
    h2 = [u2*cos(psi2)-a2*w2*sin(psi2);
           u2*sin(psi2)+a2*w2*cos(psi2)];    
 
%%%%%%%%%%%%%%%%%%%%% Camada do Ambiente %%%%%%%%%%%%%%%%%%%%%
    %atualizar as posições dos robôs 
    %aqui usar Ts aqui para multiplicar nas velocidades e ver as novas
    %coordenadas (x,y,psi) de cada robo    
	psi1 = psi1 + w1*Ts;
	psi2 = psi2 + w2*Ts;
	x1(counter+2) = h1(1)*Ts + x1(counter+1);
	y1(counter+2) = h1(2)*Ts + y1(counter+1);
	x2(counter+2) = h2(1)*Ts + x2(counter+1);
	y2(counter+2) = h2(2)*Ts + y2(counter+1);
    
    
    plot(x1(counter+1),y1(counter+1),'b--o');
    hold on;
    plot(x2(counter+1),y2(counter+1), 'r--o');
    title('Posicionamento de Robôs Uniciclo')
    legend('Robô 1','Robô 2');
    axis([0 4.5 -3 5]);
    
    pause(Ts);
    hold off;
    
    
    v = [v1' v2'];
    counter = counter + 1; %contador de iterações        
end

%calculando tempo de percurso
fprintf('Tempo de Percurso: %i\n', counter*Ts);

fprintf('Posição Inicial - Robô 1: (%i,%i)\n', x1(1), y1(1));
fprintf('Posição final - Robô 1: (%i,%i)\n', x1(counter), y1(counter));

fprintf('Posição Inicial - Robô 2: (%i,%i)\n', x2(1), y2(1));
fprintf('Posição final - Robô 2: (%i,%i)\n', x2(counter), y2(counter));

img = figure('Name','Alaf do Nascimento Santos');
plot(x1,y1,'b--o');
hold on;
plot(x2,y2, 'r--*');
title('Trajetória Percorrida')
legend('Robô 1','Robô 2');

%plotar graficos: trajetória, erro de posição final, distância da formação
%em relação ao valor final, orientação da formação em relação ao valor final,
%velocidade linear, velocidade angular, erros nas variaveis "f"

end