function [sys,x0,str,ts] = guidance(t,x,u,flag,WP,lambda)

    switch flag,
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Inicializacao              %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        case 0
        [sys,x0,str,ts] = mdlInitializeSizes(); 

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Atualizacao dos estados de tempo contínuo              %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        case 2
        sys = mdlUpdate(t,x,u,WP,lambda);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Flags nao utilizadas                                   %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        case {4,9}
        sys = [];

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Computa a saida do bloco                               %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        case 3 
        sys = mdlOutputs(x,u,WP);
    end
end 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mdlInitializeSizes
% Return the sizes, initial conditions, 
% and sample times for the S-function.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [sys,x0,str,ts] = mdlInitializeSizes()
    % dimensoes do bloco
    sizes = simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates  = 1;
    sizes.NumOutputs     = 3;
    sizes.NumInputs      = 12;
    sizes.DirFeedthrough = 12;
    sizes.NumSampleTimes = 1;   

    sys = simsizes(sizes);
    x0 = [1]; % next waypoint

    % outros
    str = [];
    ts  = [0 0];
end 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mdlUpdate
% Difference equation for the discrete-time state
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function sys = mdlUpdate(t,x,u,WP,lambda)

% Apenas realiza a troca de waypoint, quando necessário.
    idx = x;                % waypoint atual
    q = size(WP,2);         % total de waypoints
    pos = u(10:11);         % posição
    
    d = norm(WP(1:2,idx)-pos, 2)

    if d<=lambda
        if idx==q
            fprintf('\n Alcancou waypoint: [%d]', idx);
            set_param('PegasusAutopilot', 'SimulationCommand', 'stop');        
            fprintf('\n\n Simulação encerrada!\n');   
        else
           fprintf('\n Alcancou waypoint: [%d]', idx)
            idx = idx + 1;  
        end
    end    
    sys = idx';
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mdlOutputs
% Return the block outputs.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function sys=mdlOutputs(x,u,WP)

    idx = x(1); 
    wj = WP(:,idx);         % waypoint atual
    pos = u(10:11);         % posicao da aeronave no sistema NED
    
    VT_r = wj(4);
    h_r =  wj(3);
    ref = wj(1:2) - pos(1:2);    
    LOS = wrapTo2Pi(atan2(ref(2), ref(1)));
    
    sys = [VT_r h_r LOS]';
end