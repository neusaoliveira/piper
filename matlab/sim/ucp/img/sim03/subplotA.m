
%% Aerodynamic Angles
figure('Units','inches',...
    'Position',[0 0 7 5],...
    'PaperPositionMode','auto');
%% \alpha
subplot(2,1,1);
grid on;
hold on;

plot( embY.Time, radtodeg((embY.Data(:,2))), ':', 'LineWidth', 2.0, ...
      'Color', [0 0.4470 0.7410]);

axis([0 400 1 5]);
set(gca, ...
    'Units', 'normalized',...        
    'FontUnits', 'points',...
    'FontWeight', 'normal',...
    'FontSize', 10,...
    'FontName', 'Times New Roman',...
    'XTick', 0:50:400,...
    'YTick', 1:1:5);

ylabel({'\^Angulo de Ataque (graus)'}, ...
    'FontUnits', 'points',...
    'interpreter', 'latex',...
    'FontWeight', 'normal',...
    'FontSize', 10.5,...
    'FontName', 'Times New Roman');

% legend({'Resposta N\~ao-Linear', 'Resposta Discreta',}, ...
%     'FontUnits', 'points',...
%     'interpreter', 'latex',...
%     'FontSize', 10,...
%     'FontName', 'Times New Roman',...
%     'Location', 'Best');   
hold off;
%% \beta
subplot(2,1,2);
grid on;
hold on;

plot( embY.Time, radtodeg((embY.Data(:,3))), ':', 'LineWidth', 2.0, ...
      'Color', [0 0.4470 0.7410]); 
    
axis([0 400 0 5]);
set(gca, ...
    'Units', 'normalized',...        
    'FontUnits', 'points',...
    'FontWeight', 'normal',...
    'FontSize', 10,...
    'FontName', 'Times New Roman',...
    'XTick', 0:50:400,...
    'YTick', 0:1:5);

ylabel({'\^Angulo de Derrapagem (graus)'}, ...
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

% legend({'Resposta N\~ao-Linear', 'Resposta Discreta', 'Refer\^encia'}, ...
%     'FontUnits', 'points',...
%     'interpreter', 'latex',...
%     'FontSize', 10,...
%     'FontName', 'Times New Roman',...
%     'Location', 'Best');   
hold off;
%%
print -depsc2 trajA_subplotA.eps 