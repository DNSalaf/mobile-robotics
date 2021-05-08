function task1()
%TASK 1
%Forma��o com dois rob�s Pioneer 3-DX (uniciclo).

%%%%%%%%%%%%%%%%%%%%% Camada de planejamento e inicializa��o de vari�veis %%%%%%%%%%%%%%%%%%%%%
%vari�veis de forma��o
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

Ts = 100e-3; %O per�odo de amostragem considerado deve ser de 100 ms


v1 = [120 120 120 120];
v2 = [0.48 0.48 0.48 0.48];
L = diag(v1);
k = diag(v2);

q_des = [xf yf rho_f a_f]'; %n�o varia no tempo, pois � tarefa de posicionamento

%%%%%%%%%%%%%%%%%%%%% Camada de Controle %%%%%%%%%%%%%%%%%%%%%
q_erro = q_des; %inicializa��o do erro
counter = 0;
while abs(norm(q_erro)) > 1 %qual melhor forma de analizar o erro?
    q = [(x1+x2)/2 (y1+y2)/2 sqrt((x2 - x1)^2 + (y2 - y1)^2) atan((y2 - y1)/(x2 - x1))]'; %vai ser a realimenta��o
    q_erro = q_des - q; %deve decair para zero, mas nunca chegar a zero
    q_ref = L*tanh(inv(L)*k*q_erro);

    J_inv = [1 0 (-cos(a_f)/2) (rho_f*sin(a_f)/2);
             0 1 (-sin(a_f)/2) (-rho_f*cos(a_f)/2);
             1 0 (cos(a_f)/2) (-rho_f*sin(a_f)/2);
             0 1 (sin(a_f)/2) (rho_f*cos(a_f)/2)]; %jacobiano � fixo?

    x_ref = J_inv*q_ref;

    K_inv = [cos(psi1) sin(psi1) 0 0;
             (-sin(psi1)/a1) (cos(psi1)/a1) 0 0;
             0 0 cos(psi2) sin(psi2);
             0 0 (-sin(psi2)/a2) (cos(psi2)/a2)];

    v_ref = K_inv*x_ref; %sera? - contem as velocidades lineares e angulares
    

%%%%%%%%%%%%%%%%%%%%% Camada dos Rob�s %%%%%%%%%%%%%%%%%%%%%
%a partir daqui, preciso passar vref1 e vref2 pelo compensador din�mico,


    h1 = [1 3];
    h2 = [3 3];
%%%%%%%%%%%%%%%%%%%%% Camada do Ambiente %%%%%%%%%%%%%%%%%%%%%
    %atualizar as posi��es dos rob�s (for�ado nos valores teoricos por hora)
    %aqui usar Ts aqui para multiplicar nas velocidades e ver as novas
    %coordenadas (x,y) de cada robo 
    x1 = h1(1);
    y1 = h1(2);
    x2 = h2(1);
    y2 = h2(2);
    
    counter = counter + 1; %contador de itera��es
end

%calculando tempo de percurso
T_percurso = counter*Ts %dado em segundos













%plotar graficos: trajet�ria, erro de posi��o final, dist�ncia da forma��o
%em rela��o ao valor final, orienta��o da forma��o em rela��o ao valor final,
%velocidade linear, velocidade angular, erros nas variaveis "f"

end