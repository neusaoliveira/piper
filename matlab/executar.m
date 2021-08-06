clc; clear; close all;

%% Carregar Compilador
setenv('MW_MINGW64_LOC','c:\TDM-GCC-64')


%% Carregar Piloto Automático Latero-Direcional
cd sim\lat\ap;
heading_hold;
%%

%% Carregar Piloto Automático Longitudinal
cd ..\..\lon\ap;
altitude_hold;
speed_plus_pitch_hold;
%%

%% Volta ao Diretório Inicial
clc; close all;
cd ..\..\..;
%%

%% Carregar utl
cd lib\utl
addpath(pwd);
cd ..\..\;

%% Waypoints(NED)
lambda = 10;

R = 1000;
c1 = R*cos(2*pi/5);
c2 = R*cos(pi/5);
s1 = R*sin(2*pi/5);
s2 = R*sin(4*pi/5);

WP = [ 0    0    400  20; 
	   R    0    400  20; 
	   -c2  s2   420  20; 
	   c1   -s1  380  20; 
	   c1   s1   420  20; 
	   -c2  -s2  380  20; 
	   R    0    400  20]';

% WP = [ 0    0    400  20; 
% 	   c1   s1   400  20; 
% 	   -c2  s2   425  20; 
% 	   c1   -s1  375  20; 
% 	   -c2  -s2  350  20; 
% 	   c1   s1   400  20]';

% WP = [ 0    0   400  20; 
% 	   R    0   400  20; 
% 	  c1   s1   400  20; 
% 	  -c2  s2   400  20; 
% 	  -c2  -s2  400  20; 
% 	  c1   -s1  400  20; 
% 	   R    0   400  20]';

% WP = [0     0    400  20; 
% 	 +800  +800  400  20; 
% 	 -800  +800  425  20; 
% 	 +800  -800  375  20; 
% 	 -800  -800  350  20; 
% 	 +800  +800  400  20;
%       ]';    

% WP = [0     0    400  20; 
% 	 +800   0    400  20; 
% 	  0    +800  400  20; 
% 	 -800   0    400  20; 
% 	  0    -800  400  20; 
% 	 +800   0    400  20; 
%       ]';
%%

cd sim\ucp\nln
open PegasusAutopilot;
set_param('PegasusAutopilot','AlgebraicLoopSolver','LineSearch');