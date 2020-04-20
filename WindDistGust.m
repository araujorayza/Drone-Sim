function W = WindDistGust(t,Ws,time)
%WINDDISTURBANCE Summary of this function goes here
%   Detailed explanation goes here
    zeta = 9; %wind velocity at 20ft (knots)
    z = 3.3; % altitude in fts (?)
    
    sigmav=0.1*zeta;
    sigmah=sigmav/(0.177+0.000823*z)^0.4;
    Lv=z;
    Lh=Lv/(0.177+0.000823*z)^1.2;
    
    init_freq = 0.1; %rad/s
    step = 0.1;
    end_freq = 1.5; %rad/s
    
    omega = init_freq:step:end_freq;
    gammas = rand([1 length(omega)]);
   
    Phih = 0*omega;
    
    for i=1:length(omega)
        Phih(i)=sigmah^2*2*(Lh/pi)*1/(1+(Lh*omega(i))^2);
    end 
    
    a = zeros(length(omega)-1);
    for i=1:length(omega)-1
        delta_omega = omega(i+1)-omega(i);
        a(i)= sqrt(delta_omega*Phih(i));
    end 
    
    
    
    StartTime = time(1);
    EndTime = time(end);
    
    W = zeros(4,1);
    
    if(t >= StartTime && t <= EndTime)
        for i = 1:length(a)
            W = W + a(i)*sin(omega(i)*t+gammas(i));
        end

        W = W + Ws;
    end 
   
end

