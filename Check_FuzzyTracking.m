%% SCRIPT FOR TESTING THE TRACKING OF TSFUZZY MODEL

%% Check Nonzero entries of the simulation results
for i = VelX:PosYaw
    ERR = ERROR(:,i)+ DES_STATE(:,i) ...
        - STATE(:,i);
    if(CheckError(ERR,tol))
        fprintf('\n-->%dth COMPONENT NOT TRACKING\n\tErrors between %.2e and %.2e',i,min(ERR),max(ERR))
    else
        fprintf('\n-->%dth component ok',i)
    end
end
 
fprintf('\n')
%% Function
function err = CheckError(ERR,tol)
    if(find(ERR)) %if it finds nonzero error entries
        if(ERR == round(ERR,tol))
            err = true; %if it has been rounded, then
                        % the errors are real
        else
            ERR = round(ERR,tol); %if not, round and 
                                  %check for errors 
                                  %again
            err = CheckError(ERR,tol);
        end
    else
        err=false;
    end
end 