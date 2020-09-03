%% SCRIPT FOR TESTING THE TRACKING OF TSFUZZY MODEL

%% Check Nonzero entries of the simulation results
ERRx = ERROR(:,PosX)+ DES_STATE(:,PosX) ...
     - STATE(:,PosX);
 if(CheckError(ERRx,tol))
    fprintf('\n-->X NOT TRACKING\n\tErrors between %.2e and %.2e', min(ERRx), max(ERRx))
 else
	fprintf('\n-->X ok')
 end 

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