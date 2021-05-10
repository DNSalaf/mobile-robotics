%Trabalho de Rob�tica M�vel (Simula��o)
%Aluno: Alaf do Nascimento Santos
clear all, close all, clc;

%% INTERFACE COM USUARIO
prompt = {'Gostaria de simular a tarefa 1? S/N', 'Gostaria de simular a tarefa 2? S/N'}; %titulos das caixas de texto edit�vel
dims = [1 50]; %dimens�o da caixa de texto
entrada = inputdlg(prompt,'Escolha de Sa�das',dims); %variavel que guarda as entradas
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

if(R1 == 83 || R1 == 115) %S = 83, N = 78, s = 115, n = 110
    [x1,x2,y1,y2,time,norm_erro_vec,xf_erro_vec, yf_erro_vec,rho_f_erro_vec,a_f_erro_vec,veloc_lin_vec1,veloc_lin_vec2,veloc_ang_vec1,veloc_ang_vec2] = task1()  

    tempo_total = time(end);
    
    fprintf('Tempo de Percurso: %i\n', tempo_total);
    fprintf('Posi��o Inicial - Rob� 1: (%i,%i)\n', x1(1), y1(1));
    fprintf('Posi��o final - Rob� 1: (%i,%i)\n', x1(end), y1(end));
    fprintf('Posi��o Inicial - Rob� 2: (%i,%i)\n', x2(1), y2(1));
    fprintf('Posi��o final - Rob� 2: (%i,%i)\n', x2(end), y2(end));

    figure
    plot(x1,y1,'b--o');
    hold on;
    plot(x2,y2, 'r--*');
    title('Trajet�ria Percorrida')
    legend('Rob� 1','Rob� 2');
    xlabel('x[m]') 
    ylabel('y[m]') 
    grid on


    %plotagem de erros e velocidades
    figure
    subplot(2,1,1);
    plot(norm_erro_vec,time)
    grid on
    title('Norma dos Erros')
    subplot(2,1,2);
    plot(xf_erro_vec,time)
    hold on
    plot(yf_erro_vec,time)
    plot(rho_f_erro_vec,time)
    plot(a_f_erro_vec,time)
    title('Erros')
    legend('xf','yf','pf','af');
    xlabel('Tempo') 
    ylabel('Erro') 
    grid on

    figure
    subplot(2,1,1);
    plot(veloc_lin_vec1,time)
    hold on
    plot(veloc_lin_vec2,time)
    title('Velocidades Lineares');
    legend('Rob� 1','Rob� 2');
    xlabel('t[s]') 
    ylabel('u[m/s]')
    grid on

    subplot(2,1,2);
    plot(veloc_ang_vec1,time) 
    hold on
    plot(veloc_ang_vec2,time)
    title('Velocidades Angulares');
    legend('Rob� 1','Rob� 2');
    xlabel('t[s]') 
    ylabel('w[graus/s]') 
    grid on

end

if(R2 == 83 || R2 == 115) %S = 83, N = 78, s = 115, n = 110
    task2()
end
