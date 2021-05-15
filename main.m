%Trabalho de Robótica Móvel (Simulação)
%Aluno: Alaf do Nascimento Santos
clear all, close all, clc;

%% INTERFACE COM USUARIO
prompt = {'Gostaria de simular a tarefa 1 com animação? S/N','Gostaria de simular a tarefa 2? S/N'}; %titulos das caixas de texto editável
dims = [1 50]; %dimensão da caixa de texto
entrada = inputdlg(prompt,'Escolha de Saídas',dims); %variavel que guarda as entradas
R1 = double(entrada{1});
R2 = double(entrada{2});

time = zeros(1,4);
norm_erro_vec = zeros(1,4);
xf_erro_vec = zeros(1,4);
yf_erro_vec = zeros(1,4);
veloc_lin_vec1 = zeros(1,4);
veloc_lin_vec2 = zeros(1,4);
veloc_ang_vec1 = zeros(1,4);
veloc_ang_vec2 = zeros(1,4);

animado1 = false;
animado2 = false;

if(R1 == 83 || R1 == 115) %S = 83, N = 78, s = 115, n = 110
    prompt = {'Quer a animação da tarefa 1? S/N'};
    dims = [1 50];
    entrada = inputdlg(prompt,'Escolha de Saídas',dims); %variavel que guarda as entradas
    Rx = double(entrada{1});
    %se animado = true, roda animação. Se animado = fase, roda sem animação.
    if(Rx == 83 || Rx == 115)
        animado1 = true;
    end
end

if(R2 == 83 || R2 == 115) %S = 83, N = 78, s = 115, n = 110
    prompt = {'Quer a animação da tarefa 2? S/N'};
    dims = [1 50];
    entrada = inputdlg(prompt,'Escolha de Saídas',dims); %variavel que guarda as entradas
    Rx = double(entrada{1});
    %se animado = true, roda animação. Se animado = fase, roda sem animação.
    if(Rx == 83 || Rx == 115)
        animado2 = true;
    end
end

if(R1 == 83 || R1 == 115) %S = 83, N = 78, s = 115, n = 110
   
    [x1,x2,y1,y2,time,norm_erro_vec,xf_erro_vec, yf_erro_vec,rho_f_erro_vec,a_f_erro_vec,veloc_lin_vec1,veloc_lin_vec2,veloc_ang_vec1,veloc_ang_vec2] = task1(animado1)  

    tempo_total = time(end);
    
    fprintf('Tempo de Percurso: %i\n', tempo_total);
    fprintf('Posição Inicial - Robô 1: (%i,%i)\n', x1(1), y1(1));
    fprintf('Posição final - Robô 1: (%i,%i)\n', x1(end), y1(end));
    fprintf('Posição Inicial - Robô 2: (%i,%i)\n', x2(1), y2(1));
    fprintf('Posição final - Robô 2: (%i,%i)\n', x2(end), y2(end));

    figure('Name','Tarefa 1');
    plot(x1,y1,'b--o');
    hold on;
    plot(x2,y2, 'r--o');
    title('Trajetória Percorrida')
    legend('Robô 1','Robô 2');
    xlabel('x[m]') 
    ylabel('y[m]')
    grid on

    figure('Name','Tarefa 1');
    subplot(2,1,1);
    plot(time,norm_erro_vec)
    grid on
    title('Norma dos Erros')
    subplot(2,1,2);
    plot(time,xf_erro_vec)
    hold on
    plot(time,yf_erro_vec)
    plot(time,rho_f_erro_vec)
    plot(time,a_f_erro_vec)
    title('Erros')
    legend('xf','yf','pf','af');
    xlabel('Tempo') 
    ylabel('Erro') 
    grid on

    figure('Name','Tarefa 1');
    subplot(2,1,1);
    plot(time,veloc_lin_vec1)
    hold on
    plot(time,veloc_lin_vec2)
    title('Velocidades Lineares');
    legend('Robô 1','Robô 2');
    xlabel('t[s]') 
    ylabel('u[m/s]')
    grid on

    subplot(2,1,2);
    plot(time,veloc_ang_vec1) 
    hold on
    plot(time,veloc_ang_vec2)
    title('Velocidades Angulares');
    legend('Robô 1','Robô 2');
    xlabel('t[s]') 
    ylabel('w[graus/s]') 
    grid on
end

if(R2 == 83 || R2 == 115) %S = 83, N = 78, s = 115, n = 110
    [x1,x2,y1,y2,z1,z2,time,norm_erro_vec,xf_erro_vec, yf_erro_vec,zf_erro_vec,rho_f_erro_vec,a_f_erro_vec,b_f_erro_vec,veloc_lin_vec1,veloc_lin_vec2] = task2(animado2);
    
    tempo_total = time(end);
    
    fprintf('Tempo de Percurso: %i\n', tempo_total);
    fprintf('Posição Inicial - Robô 1: (%i,%i,%i)\n', x1(1), y1(1),z1(1));
    fprintf('Posição final - Robô 1: (%i,%i,%i)\n', x1(end), y1(end),z1(end));
    fprintf('Posição Inicial - Robô 2: (%i,%i,%i)\n', x2(1), y2(1),z2(1));
    fprintf('Posição final - Robô 2: (%i,%i,%i)\n', x2(end), y2(end),z2(end));

    figure('Name','Tarefa 2');
    plot3(x1,y1,z1,'b--o');
    title('Trajetória Percorrida')
    xlabel('x[m]');
    ylabel('y[m]');
    zlabel('Z[m]')
    grid on
    hold on
    plot3(x2,y2,z2,'r--o');

    figure('Name','Tarefa 2');
    subplot(2,1,1);
    plot(time,norm_erro_vec)
    grid on
    title('Norma dos Erros')
    subplot(2,1,2);
    plot(time,xf_erro_vec)
    hold on
    plot(time,yf_erro_vec)
    plot(time,zf_erro_vec)
    plot(time,rho_f_erro_vec)
    plot(time,a_f_erro_vec)
    plot(time,b_f_erro_vec)
    title('Erros')
    legend('xf','yf','zf','pf','af','bf');
    xlabel('Tempo') 
    ylabel('Erro') 
    grid on

    figure('Name','Tarefa 2');
    plot(time,veloc_lin_vec1)
    hold on
    plot(time,veloc_lin_vec2)
    title('Velocidades Lineares');
    legend('Robô 1','Robô 2');
    xlabel('t[s]') 
    ylabel('u[m/s]')
    grid on
end
