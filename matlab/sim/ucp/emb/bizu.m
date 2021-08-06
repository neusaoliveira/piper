% Load data
equilibrium;
% load dataEMB100;
% load dataNL100;

%% Trajectory
figure('Units','inches',...
    'Position',[1 1 7 5],...
    'PaperPositionMode','auto');
grid on;
hold on;    

plot( embY.Data(:,10), embY.Data(:,11), ':', 'LineWidth', 2.0, ...
      'Color', [0 0.4470 0.7410]);

% axis([-1000 500 -1500 1500]);
% set(gca, ...
%     'Units', 'normalized',...        
%     'FontUnits', 'points',...
%     'FontWeight', 'normal',...
%     'FontSize', 10,...
%     'FontName', 'Times New Roman',...
%     'XTick', -1000:250:500,...
%     'YTick', -1500:500:1500,...
%     'Position',[.15 .2 .75 .7]);

ylabel({'Leste (m)'}, ...
    'FontUnits', 'points',...
    'interpreter', 'latex',...
    'FontWeight', 'normal',...
    'FontSize', 10.5,...
    'FontName', 'Times New Roman');

xlabel('Norte (m)',...
    'FontUnits', 'points',...
    'interpreter', 'latex',...
    'FontWeight', 'normal',...
    'FontSize', 10.5,...
    'FontName', 'Times New Roman');
hold off;


%% \Vt
figure('Units','inches',...
    'Position',[1 1 7 5],...
    'PaperPositionMode','auto');
grid on;
hold on;

plot( embY.Time, embY.Data(:,1), ':', 'LineWidth', 2.0, ...
      'Color', [0 0.4470 0.7410]);
    
% axis([0 400 16 24]);
% set(gca, ...
%     'Units', 'normalized',...        
%     'FontUnits', 'points',...
%     'FontWeight', 'normal',...
%     'FontSize', 10,...
%     'FontName', 'Times New Roman',...
%     'XTick', 0:50:400,...
%     'YTick', 16:2:24,...
%     'Position',[.15 .2 .75 .7]);

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
hold off;

%% H
figure('Units','inches',...
    'Position',[1 1 7 5],...
    'PaperPositionMode','auto');
grid on;
hold on; 

plot( embY.Time, embY.Data(:,12), ':', 'LineWidth', 2.0, ...
      'Color', [0 0.4470 0.7410]);

% axis([0 400 340 440]);
% set(gca, ...
%     'Units', 'normalized',...        
%     'FontUnits', 'points',...
%     'FontWeight', 'normal',...
%     'FontSize', 10,...
%     'FontName', 'Times New Roman',...
%     'XTick', 0:50:400,...
%     'YTick', 340:20:440,...
%     'Position',[.15 .2 .75 .7]);

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
hold off;

%% \psi
figure('Units','inches',...
    'Position',[1 1 7 5],...
    'PaperPositionMode','auto');
grid on;
hold on; 

plot( embY.Time, radtodeg((embY.Data(:,9))), ':', 'LineWidth', 2.0, ...
      'Color', [0 0.4470 0.7410]);

% axis([0 400 0 360]);
% set(gca, ...
%     'Units', 'normalized',...        
%     'FontUnits', 'points',...
%     'FontWeight', 'normal',...
%     'FontSize', 10,...
%     'FontName', 'Times New Roman',...
%     'XTick', 0:50:400,...
%     'YTick', 0:90:360,...
%     'Position',[.15 .2 .75 .7]);

ylabel({'\^Angulo de Guinada (graus)'}, ...
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
hold off;