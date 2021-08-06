% pitch_attitude_hold;

%% Ganhos Controle
% Inner Loops
KVt = 1.00;
% PIs
Kp_Vt = 0.00;
Ki_Vt = 0.00;
%%

%% Carregar modelo
Bvt = [aeronave.B(:,1); 0; 0];
olvt = ss(clt.A, Bvt, clt.C, 0);
%%

%% Speed Hold
% Vt
% Select I/O pair.
GVt = zpk(olvt(1,1))
%pidtool(GVt);
Kp_Vt = 0.030269;
Ki_Vt = 0.022003;
CVt = pid(Kp_Vt, Ki_Vt);

% Feedback [Vt]
clv = feedback(series(olvt, CVt), KVt, 1, 1);
disp('Vt Damper');
damp(clv);
clearvars Bvt olvt Kp_Vt Ti_Vt;
%%