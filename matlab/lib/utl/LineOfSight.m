function LOS = LineOfSight(y, x, psi)
    LOS = atan2(y, x);        

    if abs(LOS - psi) > pi
        turn = fix(psi/(2*pi));
        psiR = LOS + 2*pi*turn;
        if abs(psiR - psi) > pi
            aux = psiR - 2*pi; 
            if abs(aux - psi) < pi
                LOS = aux;
            else
                aux = psiR + 2*pi;
                if abs(aux - psi) < pi
                    LOS = aux;
                end
            end
        else
            LOS = psiR;
        end
    end
end