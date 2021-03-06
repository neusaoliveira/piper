clc; close all;

%%
cd ..\..\..\..\img
addpath(pwd);
cd ..\sim\lon\img\pth
%%

%% Load data
load data;
load dataNL100;

% Carrega entradas e estados de equilibrio;
equilibrium;

% % co=[  0       0.4470    0.7410
% %     0.8500    0.3250    0.0980
% %     0.9290    0.6940    0.1250
% %     0.4940    0.1840    0.5560
% %     0.4660    0.6740    0.1880
% %     0.3010    0.7450    0.9330
% %     0.6350    0.0780    0.1840];
% % 
% % co=get(0, 'DefaultAxesColorOrder');
% % set(0,'DefaultAxesColorOrder',co([1 7 5 4 2 6 3],:));

% %% Pitch Attitude Hold
% % figure(6);   
% %     hold all;
% %     grid on;
% %     plot( U_pitch.Time, radtodeg((U_pitch.Data)+Ue(2)),'-', 'LineWidth', 1.5);
% %     plot( NLU_pitch.Time, radtodeg(NLU_pitch.Data(:,2)),'-.','LineWidth', 1.5);
% %     legend('Linear response', 'Non-Linear response'); 
% %     xlabel('Time (s)');
% %     ylabel('\delta_e (?)');    
     
%% Vt
% figure('Units','inches',...
%     'Position',[1 1 7 3],...
%     'PaperPositionMode','auto');
figure('Units','inches',...
    'Position',[1 1 5.04 3.78],...
    'PaperPositionMode','auto');
grid on;
hold on;

plot( Y.Time, Y.Data(:,1)+norm(Xe(1:3)),'-', 'LineWidth', 2.0);
plot( NLY.Time, NLY.Data(:,1),'-.','LineWidth', 2.0);

axis([0 60 14 22]);
set(gca, ...
    'Units', 'normalized',...        
    'FontUnits', 'points',...
    'FontWeight', 'normal',...
    'FontSize', 10,...
    'FontName', 'Times New Roman',...
    'XTick', 0:10:60,...
    'YTick', 14:2:22,...
    'Position',[.15 .2 .75 .7]);

ylabel({'Velocidade Aerodin\^amica (m/s)'}, ...
    'FontUnits', 'points',...
    'interpreter', 'latex',...
    'FontWeight', 'normal',...
    'FontSize', 10.5,...
    'FontName', 'Times New Roman');

xlabel('Tempo (s)',...
    'FontUnits', 'points',...
    'interpreter', 'latex',...
    'FontWeight', 'normal',...
    'FontSize', 10.5,...
    'FontName', 'Times New Roman');

legend({'Resposta Linear', 'Resposta N\~ao Linear'}, ...
    'FontUnits', 'points',...
    'interpreter', 'latex',...
    'FontSize', 10,...
    'FontName', 'Times New Roman',...
    'Location', 'Best');   

print -depsc2 pth_vt_beamer.eps
hold off;
     
%% \theta
figure('Units','inches',...
    'Position',[1 1 5.04 3.78],...
    'PaperPositionMode','auto');
grid on;
hold on;    

plot( Y.Time, radtodeg(Y.Data(:,4)+Xe(8)), '-','LineWidth', 2.0);
plot( NLY.Time, radtodeg(NLY.Data(:,4)), '-.','LineWidth', 2.0);
plot( NLstep.Time, radtodeg(NLstep.Data), '--k','LineWidth', 1.5);  

axis([0 60 2 10]);
set(gca, ...
    'Units', 'normalized',...        
    'FontUnits', 'points',...
    'FontWeight', 'normal',...
    'FontSize', 10,...
    'FontName', 'Times New Roman',...
    'XTick', 0:10:60,...
    'YTick', 2:2:10,...
    'Position',[.15 .2 .75 .7]);

ylabel({'\^Angulo de Arfagem (graus)'}, ...
    'FontUnits', 'points',...
    'interpreter', 'latex',...
    'FontWeight', 'normal',...
    'FontSize', 10.5,...
    'FontName', 'Times New Roman');

xlabel('Tempo (s)',...
    'FontUnits', 'points',...
    'interpreter', 'latex',...
    'FontWeight', 'normal',...
    'FontSize', 10.5,...
    'FontName', 'Times New Roman');

legend({'Resposta Linear', 'Resposta N\~ao Linear', 'Refer\^encia'}, ...
    'FontUnits', 'points',...
    'interpreter', 'latex',...
    'FontSize', 10,...
    'FontName', 'Times New Roman',...
    'Location', 'Best');   
hold off;
print -depsc2 pth_theta_beamer.eps


%% h
figure('Units','inches',...
    'Position',[1 1 5.04 3.78],...
    'PaperPositionMode','auto');
grid on;
hold on;

plot( Y.Time, Y.Data(:,5)-Xe(12), '-','LineWidth', 2.0);
plot( NLY.Time, NLY.Data(:,5), '-.','LineWidth', 2.0);

axis([0 60 400 440]);
set(gca, ...
    'Units', 'normalized',...        
    'FontUnits', 'points',...
    'FontWeight', 'normal',...
    'FontSize', 10,...
    'FontName', 'Times New Roman',...
    'XTick', 0:10:60,...
    'YTick', 400:10:440,...
    'Position',[.15 .2 .75 .7]);

ylabel({'Altitude (m)'}, ...
    'FontUnits', 'points',...
    'interpreter', 'latex',...
    'FontWeight', 'bold',...
    'FontSize', 10.5,...
    'FontName', 'Times New Roman');

xlabel('Tempo (s)',...
    'FontUnits', 'points',...
    'interpreter', 'latex',...
    'FontWeight', 'bold',...
    'FontSize', 10.5,...
    'FontName', 'Times New Roman');

legend({'Resposta Linear', 'Resposta N\~ao Linear'}, ...
    'FontUnits', 'points',...
    'interpreter', 'latex',...
    'FontSize', 10,...
    'FontName', 'Times New Roman',...
    'Location', 'Best');   
hold off;
print -depsc2 pth_h_beamer.eps

%%
