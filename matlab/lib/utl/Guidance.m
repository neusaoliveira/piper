function [sys,x0,str,ts] = Guidance(t,x,u,flag,WP,lambda)

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
    sizes.NumDiscStates  = 3;
    sizes.NumOutputs     = 3;
    sizes.NumInputs      = 12;
    sizes.DirFeedthrough = 12;
    sizes.NumSampleTimes = 1;   

    sys = simsizes(sizes);
    x0 = [1 0 1];             %idxWP psiRef phases

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
    idx  = x(1);             % waypoint atual
    psiR = x(2);
    phases = x(3);
    
    wj = WP(:,idx+1);
    pos_r = wj(1:2) - u(10:11);         
    
    switch phases       
            
        case 1             
            psiR = LineOfSight(pos_r(2), pos_r(1), u(9));
            if radtodeg(abs(u(9) - psiR)) <= 5.0
                phases = phases + 1;
            end
            
        case 2
            if (WP(1,idx+1) - WP(1,idx)) ~= 0
                m  = (WP(2,idx+1) - WP(2,idx)) / (WP(1,idx+1) - WP(1,idx));
                er = abs( (u(11) - m*u(10) - (WP(2,idx) - m*WP(1,idx)) ) / (sqrt(m^2+1)) );
                Nr =  u(10) - er*u(9);
                Er =  u(11) - er*u(9);

                d = norm(WP(1:2,idx+1)-([Nr; Er]), 2)
                ds = norm(WP(1:2,idx+1)-WP(1:2,idx), 2);
                eMax = (d*((70+40)*0.5))/ds;

                if er > eMax 
                    psiR = LineOfSight(pos_r(2), pos_r(1), u(9));
                end 
            else
                psiR = LineOfSight(pos_r(2), pos_r(1), u(9));
                d = norm(pos_r, 2);
            end               
            
            if d<=lambda                
                fprintf('\nAlcancou waypoint: [%d]', idx)
                idx = idx + 1;
                phases = 1;                
            end   
    end
    
    if idx==size(WP,2)
        fprintf('\nAlcancou waypoint: [%d]', idx);
        fprintf('\n\nSimulação encerrada!\n');
        set_param('PegasusAutopilot', 'SimulationCommand', 'stop');             
    end
    
    sys = [idx psiR phases]';
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mdlOutputs
% Return the block outputs.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function sys=mdlOutputs(x,u,WP)   
    
    if x(3) == 2 
        wj = WP(:, x(1)+1);                  % proximo waypoint 
        VT_r = wj(4);
        h_r =  wj(3);
    else
        wj = WP(:, x(1));                % waypoint atual 
        VT_r = wj(4);
        h_r =  wj(3);
    end   
        
    psi_r = x(2);    
    sys = [VT_r h_r psi_r]';
end