function U = CalcVirtControlLaw(Controller,t,State,DesState)
    K=Controller.Gain;
    
    switch Controller.type
        case 'StateFeedback'
            
            U=-K*(State - DesState);%this control law was designed for the  
                                    %error dynamics so the 'state' it uses
                                    %is the error in position and velocity
            
        case 'Fuzzy'
            min=1;
            max=2;
            
            Z = Controller.Fuzzy_Z;
            z = Controller.Fuzzy_z;
            
            psi = State(8);
            
            for i=1:length(z)
                M(i,1) = (Z(i,max) - z{i}(psi))/(Z(i,max)-Z(i,min));
                M(i,2) = 1 - M(i,1);
            end
            
            h=CalcDefuzzWeights(M);
            
            U = [0;0;0;0];
            for i=1:length(K)
               U = U - K{i,1}*h(i)*(State - DesState);
                                    %this control law was designed for the  
                                    %error dynamics so the 'state' it uses
                                    %is the error in position and velocity
            end
            
        case 'OpenLoop'
           U=zeros(4,length(t)); 
        otherwise
           U=zeros(4,length(t)); 
    end


end