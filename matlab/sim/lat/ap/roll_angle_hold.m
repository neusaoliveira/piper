yaw_damper;

%% Ganhos Controle
Kphi = 1.00;
% PIs
Kp_phi = 0.00;
Ki_phi = 0.00;
%%

%% Roll-Angle Hold
% Select I/O pair.
Gphi = zpk(clr('\phi','\delta_a')) 
%pidtool(Gphi);
Kp_phi = -0.090681;
Ki_phi = -0.019958;
Cphi = pid(Kp_phi, Ki_phi);

% figure;
%     set(gcf,'DefaultLineLineWidth', 1.5);
%     set(gcf,'DefaultlineMarkerSize', 10);
%     rlocus(Gphi);
%     renameRLocus;
%     rphi = rlocus(Gphi*Cphi, Kphi);
%     %axis([-10, 5, -10, 10]);
%     hold on;
%         plot(rphi+eps*j, 'rd', 'MarkerFace', 'r');
%     hold off;

% [Outer Loop (Phi)]
clphi = feedback(series(clr, Cphi), Kphi, 1, 4);
disp('Roll Angle Hold');
damp(clphi);
clearvars rphi Kp_phi Ti_phi
%%