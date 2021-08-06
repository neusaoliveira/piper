% clc; clear; close all; 

%%
cd ..\..\..\img
addpath(pwd);
cd ..\sim\lon\ap
%%

%% Ganhos Controle
% Inner Loops
Kq = 1.00;
Ktheta = 1.00;
% PIs
Kp_theta = 0.00;
Ki_theta = 0.00;
%%

%% Carregar modelo
cd ..\..\..\mdl
addpath(pwd);

% Modelo Longitudinal
load('Piper J3 1_4 VT0_20.mat');

[A, B, C, D] = decoupling(1, par_gen, par_aero, par_prop);
states = {'u' 'w' 'q' '\theta' 'h'};
inputs = {'\delta_t' '\delta_e'};
outputs = {'Vt' '\alpha' 'q' '\theta' 'h'};
aeronave = ss(A, B, C , D, 'statename',states,...
                           'inputname',inputs,...
                           'outputname',outputs);
clearvars A B C D;

% Carrega entradas e estados de equilibrio;
equilibrium;

% Dinamica Longitudinal do Vant 
damp(aeronave);

cd ..\sim\lon\ap
%%

%% Stability Augmentation System (SAS)
% Pitch Damper
% Select I/O pair.
Gq = zpk(aeronave('q','\delta_e')) 
figure;
    % root locus: zeta = 0.7, wn = 15.6, gain = 0.0877
    set(gcf,'DefaultLineLineWidth', 1.5);
    set(gcf,'DefaultlineMarkerSize', 10);
    Kq = 0.0877;
    rlocus(Gq, linspace(0, 1, 10000));
    renameRLocus;
    rq = rlocus(Gq, Kq);    
    sgrid(0.7, 0);
    axis([-20, 5, -15, 15]);
    hold on; 
        plot(rq+eps*j,'rd','MarkerFace','r');
    hold off
        
% [Inner Loop (q)]
cl = feedback(aeronave, Kq, 2, 3);
disp('Pitch Damper');
damp(cl);
%%

%% Pitch-Attitude Hold
% Select I/O pair.
Gtheta = zpk(cl('\theta','\delta_e'))
%pidtool(Gtheta);
Kp_theta = -0.45319;
Ki_theta = -0.66623;
Ctheta = pid(Kp_theta, Ki_theta);

% figure;
%     set(gcf,'DefaultLineLineWidth', 1.5);
%     set(gcf,'DefaultlineMarkerSize', 10);
%     rlocus(Gtheta);
%     renameRLocus;
%     rtheta = rlocus(Gtheta*Ctheta, Ktheta);
%     axis([-20, 5, -20, 20]);
%     hold on;
%         plot(rtheta+eps*j, 'rd', 'MarkerFace', 'r');
%     hold off;
    
% [Outer Loop (Theta)]
clt = feedback(series(cl, Ctheta), Ktheta, 2, 4);
disp('Pitch Attitude Hold');
damp(clt);
clearvars rq rtheta Kp_theta Ti_theta;
%%