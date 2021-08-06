% propulsion model
% INPUTS:
% aero_var: structure with following fields
%   -> rho: air density
%   -> V: TAS
%   -> alpha: angle of attack
%   -> beta: sideslip angle
%   -> p: roll rate
%   -> q: pitch rate
%   -> r: yaw rate
%   -> ct: throttle deflection
%   -> ca: aileron deflection
%   -> ce: elevator deflection
%   -> cr: rudder deflection
% par_prop: parameters of propulsion model (generated by DataA300)
% par_aero: parameters of aerodynamic model (generated by DataA300)
function [T]=propulsion(aero_var, par_prop, par_aero)

T = zeros(3,1);

% F_thrust= F_max*d_thrust*(VT_0/VT)^nv*(rho/rho_0)^np
T(1) = par_prop.Fmax*aero_var.ct*(par_aero.Vref/aero_var.V)*(aero_var.rho/1.225)^0.8; 

%% please, wait a [moment]
% MT ? [ 0 zf*FT*cos(af)-xf*sin(af) 0]';