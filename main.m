%Trabalho de Robótica Móvel (Simulação)
%Aluno: Alaf do Nascimento Santos
clear all, close all, clc;

%% INTERFACE COM USUARIO
prompt = {'Gostaria de simular a tarefa 1? S/N', 'Gostaria de simular a tarefa 2? S/N'}; %titulos das caixas de texto editável
dims = [1 50]; %dimensão da caixa de texto
entrada = inputdlg(prompt,'Escolha de Saídas',dims); %variavel que guarda as entradas
R1 = double(entrada{1});
R2 = double(entrada{2});

if(R1 == 83 || R1 == 115) %S = 83, N = 78, s = 115, n = 110
    tarefa1()
end

if(R2 == 83 || R2 == 115) %S = 83, N = 78, s = 115, n = 110
    tarefa2()
end
