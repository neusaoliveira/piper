%% Longitudinal Controller
figure('Units','inches',...
    'Position',[0 0 7 10],...
    'PaperPositionMode','auto');
%% \theta
subplot(3,1,2);
grid on;
hold on;

plot( NLY.Time, radtodeg(NLY.Data(:,4)), '-', 'LineWidth', 2.0);
plot( EMB_Y.Time, radtodeg(EMB_Y.Data(:,4)), ':', 'LineWidth', 2.0);

axis([0 300 -5 11]);
set(gca, ...
    'Units', 'normalized',...        
    'FontUnits', 'points',...
    'FontWeight', 'normal',...
    'FontSize', 10,...
    'FontName', 'Times New Roman',...
    'XTick', 0:50:300,...
    'YTick', -5:4:11);

ylabel({'\^Angulo de Arfagem (graus)'}, ...
    'FontUnits', 'points',...
    'interpreter', 'latex',...
    'FontWeight', 'normal',...
    'FontSize', 10.5,...
    'FontName', 'Times New Roman');

legend({'Resposta N\~ao Linear', 'Resposta Discreta',}, ...
    'FontUnits', 'points',...
    'interpreter', 'latex',...
    'FontSize', 10,...
    'FontName', 'Times New Roman',...
    'Location', 'Best'); 
%% V_T
subplot(3,1,1);
grid on;
hold on;

plot( NLY.Time, NLY.Data(:,1),'-', 'LineWidth', 2.0);
plot( EMB_Y.Time, EMB_Y.Data(:,1),':', 'LineWidth', 2.0);

axis([0 300 19.0 21.0]);
set(gca, ...
    'Units', 'normalized',...        
    'FontUnits', 'points',...
    'FontWeight', 'normal',...
    'FontSize', 10,...
    'FontName', 'Times New Roman',...
    'XTick', 0:50:300,...
    'YTick', 19.0:0.5:21.0);

ylabel({'Velocidade Aerodin\^amica (m/s)'}, ...
    'FontUnits', 'points',...
    'interpreter', 'latex',...
    'FontWeight', 'normal',...
    'FontSize', 10.5,...
    'FontName', 'Times New Roman');

legend({'Resposta N\~ao Linear', 'Resposta Discreta',}, ...
    'FontUnits', 'points',...
    'interpreter', 'latex',...
    'FontSize', 10,...
    'FontName', 'Times New Roman',...
    'Location', 'Best');
%% H
subplot(3,1,3);
grid on;
hold on;

plot( NLY.Time, NLY.Data(:,5), '-', 'LineWidth', 2.0);
plot( EMB_Y.Time, EMB_Y.Data(:,5), ':', 'LineWidth', 2.0);
plot( EMBstep.Time, EMBstep.Data(:,2), '--k','LineWidth', 1.5); 

axis([0 300 362.5 437.5]);
set(gca, ...
    'Units', 'normalized',...        
    'FontUnits', 'points',...
    'FontWeight', 'normal',...
    'FontSize', 10,...
    'FontName', 'Times New Roman',...
    'XTick', 0:50:300,...
    'YTick', 362.5:12.5:437.5);

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

legend({'Resposta N\~ao Linear', 'Resposta Discreta', 'Refer\^encia'}, ...
    'FontUnits', 'points',...
    'interpreter', 'latex',...
    'FontSize', 10,...
    'FontName', 'Times New Roman',...
    'Location', 'Best');   
hold off;
%%
print -depsc2 lon_emb_subplot.eps 
%samexaxis('join','ytac','xmt','off', 'yld', 1)
%%