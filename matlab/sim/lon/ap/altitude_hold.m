pitch_attitude_hold;
 
%% Ganhos Controle
% Inner Loops
Kh = 1.00;
% PIs
Kp_h = 0.0;
Ki_h = 0.0;
%%

%% Altitude Hold
% h
% Select I/O pair.
Gh = zpk(clt(5,2))
%pidtool(zpk(Gh));
Kp_h = 0.0172410;
Ki_h = 0.0059742;
Ch = pid(Kp_h, Ki_h);

% figure;
%     set(gcf,'DefaultLineLineWidth', 1.5);
%     rlocus(Gh);
%     renameRLocus;
%     sgrid;
%     hold on;
%         plot(rh+eps*j, 'gs', 'MarkerFace', 'g');
%     hold off;

% Realimentacao de [h]
clh = feedback(series(clt, Ch), Kh, 2, 5);
disp('Altitude Hold');
damp(clh);
clearvars Aclh Bclh Cclh rh Kp_h Ti_h
%%

