%clc; close all; clear;

%%
cd ..\..\..\img
addpath(pwd);
cd ..\sim\lat\ap
%%

%% Ganhos Controle
% Inner Loops
Kp  = 1.00;
Kr  = 1.00;
tau = 1.00;
%%

%% Carregar modelo
cd ..\..\..\mdl
addpath(pwd);

% Modelo Latero-Direcional
load('Piper J3 1_4 VT0_20.mat');

[A, B, C, D] = decoupling(2, par_gen, par_aero, par_prop);
states = {'\beta' 'p' 'r' '\phi' '\psi'};
inputs = {'\delta_a' '\delta_r'};
outputs = {'\beta' 'p' 'r' '\phi' '\psi'};
aeronave = ss(A, B, C , D, 'statename',states,...
                           'inputname',inputs,...
                           'outputname',outputs);                       
clearvars A B C D; 

% Carrega entradas e estados de equilibrio;
equilibrium;

% Dinamica Latero-Direcional do Vant              
damp(aeronave);
cd ..\sim\lat\ap
%%

%% Washout Filter  
aw= [-1/tau];   bw= [0 1/tau];  
cw= [0;-1];     dw= [1 0; 0 1]; 
wash= ss(aw, bw, cw, dw, 'inputname', inputs);
planta = series(wash, aeronave);
clearvars aw bw cw dw;
%%

%% Stability Augmentation System (SAS)
% Roll Damper
% Select I/O pair.
Gp = zpk(planta('p','\delta_a')) 
figure;    
    % root locus: zeta = 0.448, wn = 5.67, gain = 0.119
    set(gcf,'DefaultLineLineWidth', 1.5);
    set(gcf,'DefaultlineMarkerSize', 10);
    Kp = 0.119;
    rlocus(Gp, linspace(0, 1, 10000));
    rp = rlocus(Gp, Kp);
    renameRLocus;
    sgrid(0.448, 0);
    axis([-55, 5, -8, 8]);    
    hold on;
        plot(rp+eps*j, 'rd', 'MarkerFace', 'r');
    hold off;
    
% Inner Loop (p)
clp = feedback(planta, Kp, 1, 2);
disp('Roll Damper');
damp(clp);

% Yaw-Rate
% Select I/O pair.
Gr = zpk(clp(3, 2))
figure;    
    % root locus: zeta = 0.707, wn = 5.33, gain = 0.105
    set(gcf,'DefaultLineLineWidth', 1.5);
    set(gcf,'DefaultlineMarkerSize', 10);
    rlocus(Gr, linspace(0, 10, 10000));
    Kr = 0.105;
    rr = rlocus(Gr, Kr);
    renameRLocus;
    sgrid(0.7, 0);
    %axis([-25, 5, -10, 10]);
    axis([-35, 5, -6, 6]);
    hold on;
        plot(rr+eps*j, 'rd', 'MarkerFace', 'r');
    hold off;
    
% Inner Loop (r)
clr = feedback(clp, Kr, 2, 3);
disp('Yaw Damper');
damp(clr);
clearvars rp rr 
%%