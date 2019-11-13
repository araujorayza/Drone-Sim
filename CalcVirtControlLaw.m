function U = CalcVirtControlLaw(Controller,t,State,DesAcc,DesVel,DesPos)
    DesState = [DesVel;DesPos];
    K=Controller.Gain;
    
    switch Controller.type
        case 'StateFeedback'
            
            U=-K*(State - DesState);%this control law was designed for the  
                                    %error dynamics so the 'state' it uses
                                    %is the error in position and velocity
            
        case 'Fuzzy'
            min=1;max=2;
            
            Z = Controller.Fuzzy_Z;
            z = Controller.Fuzzy_z;
            
            psi = State(8);
            
            for i=1:length(z)
                M(i,1) = (z{i}(psi)-Z(i,min))/(Z(i,max)-Z(i,min));
                M(i,2) = 1 - M(i,1);
            end
            
            h=CalcDefuzzWeights(M);
            
            U = [0;0;0;0];
            for i=1:length(K)
               U = U - K{1,i}*h(i)*(State - DesState);
                                    %this control law was designed for the  
                                    %error dynamics so the 'state' it uses
                                    %is the error in position and velocity
            end
            
        case 'FuzzyWithZuncertainty'    
            min=1;max=2;
            
            Z = Controller.Fuzzy_Z;
            z = Controller.Fuzzy_z;
            
            psi = State(8);
            
            for i=1:length(z)
                M(i,1) = (z{i}(psi)-Z(i,min))/(Z(i,max)-Z(i,min));
                M(i,2) = 1 - M(i,1);
            end
            
            h=CalcDefuzzWeights(M);
            
            U = [0;0;0;0];
            for i=1:length(K)/2
               U = U - K{1,i}*h(i)*(State - DesState);
                                    %this control law was designed for the  
                                    %error dynamics so the 'state' it uses
                                    %is the error in position and velocity
            end
        case 'LQR'
            U=-K*(State - DesState);
        case 'FeedbackLin'
            psi = State(8);
            N = [ gamma(2)*cos(psi), -gamma(4)*sin(psi),      0,        0;
                gamma(2)*sin(psi),  gamma(4)*cos(psi),      0,        0;
                0,                  0,              gamma(6),   0;
                0,                  0,                0,    gamma(8)];
            

            R = [ cos(psi),    -sin(psi),      0,    0;
                  sin(psi),     cos(psi),      0,    0;
                    0,          0,         1,    0;
                    0,          0,         0,    1];
            U = [N*R'-K{1}, -K{2}]*(State - DesState);
        case 'OpenLoop'
           U=zeros(4,length(t));
           U=ones(length(t)).*[sin(t)+sin(1.3*t);
                               -cos(4*t)-cos(5.3*t);
                               sin(2*t)+sin(2.3*t);
                               cos(0.5*t)+cos(0.3*t)];
                           
           disp('Virtual Control is open loop')
        otherwise
           U=zeros(4,length(t)); 
    end


end