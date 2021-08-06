roll_angle_hold;

%% Ganhos Controle
u = Xe(1);
g = 9.80655;
tau1 = 7.5;
%%

%% Carregar modelo
Kpsi = u/(tau1*g);
ol = series(clphi, Kpsi);
%%
    
%% Heading Hold 
% Select I/O pair.
Gpsi = zpk(ol(5, 1))
clpsi = feedback(ol, 1, 1, 5); 
disp('Heading Hold');
damp(clpsi);
%%

clearvars u g tau1;
