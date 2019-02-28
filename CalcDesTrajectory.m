function [q_d,dq_d,ddq_d]=CalcDesTrajectory(type,t)
    switch type
        case 'circle'
            raio = 1.5;
            
            q_d =   [raio*cos(t);
                    raio*sin(t);
                    ones(1,length(t));
                    t];
            
            dq_d =  [-raio*sin(t);
                    raio*cos(t);
                    zeros(1,length(t));
                    ones(1,length(t))];
            
            ddq_d = [-raio*cos(t);
                    -raio*sin(t);
                    zeros(1,length(t));
                    zeros(1,length(t))]; 
                
        case 'LemniscataBernoulli'
            a=1;
            
            q_d =  [a*cos(t).*sqrt(2.*cos(2*t));
                    a*sin(t).*sqrt(2.*cos(2*t));
                    ones(1,length(t));
                    zeros(1,length(t))];
            
            dq_d = [-(2^(1/2)*a.*sin(3.*t))./cos(2.*t).^(1/2);
                    (2^(1/2)*a.*cos(3.*t))./cos(2.*t).^(1/2);
                    zeros(1,length(t));
                    zeros(1,length(t))];
           
            ddq_d =[-(2^(1/2)*a.*(cos(5.*t) + 2.*cos(t)))./cos(2.*t).^(3/2);
                    -(2^(1/2)*a.*(sin(5.*t) + 2.*sin(t)))./cos(2.*t).^(3/2);
                    zeros(1,length(t));
                    zeros(1,length(t))];
        otherwise
            q_d=zeros(4,length(t));
            dq_d=zeros(4,length(t));
            ddq_d=zeros(4,length(t));
    end
end