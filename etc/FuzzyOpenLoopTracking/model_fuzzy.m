function dE = model_fuzzy(t,E)
    load gamma gamma
    [A,B,M,N,R,Z] = CalcTSModel(gamma);
    
    min=1;max=2;
            
    z = {@(psi) cos(psi);
         @(psi) sin(psi)};
%     
    [q_d,dq_d,~]=CalcDesTrajectory('circle',t);
    DES_STATE = [dq_d;q_d];
    psi = E(8) + DES_STATE(8);
%     
    H = zeros(length(z),2);
    for i=1:length(z)
        H(i,min) = (z{i}(psi)-Z(i,min))/(Z(i,max)-Z(i,min)); %this is inverted like this. 
        H(i,max) = 1 - H(i,1);
    end
    
    h=CalcDefuzzWeights(H);
%     
%     dE = zeros(8,1);
%     Nh=N{1,1}*0;
%     Rh=R{1,1}*0;
%     Bh=B{1,1}*0;
    U=ones(4,1);
%     
%     for i=1:length(h)
%         Nh = Nh + h(i)*N{1,i};
%         Rh = Rh + h(i)*R{1,i};
%         Bh = Bh + h(i)*B{1,i};
%     end
%     
    %------------------------------------------
    z10 = -1;
    z11 =  1;  
    z20 = -1; 
    z21 =  1;  
    
    M10=(cos(psi)-z10)/(z11-z10);
    M11=1-M10;
    M20=(sin(psi)-z20)/(z21-z20);
    M21=1-M20;
    
    ri=size(N,2);
%     h=zeros(1,ri);
%     h(1)=M11*M21;
%     h(2)=M11*M20;
%     h(3)=M10*M21;
%     h(4)=M10*M20;
    
    [Ln,Cn]=size(N{1,1});
    [Lm,Cm]=size(M{1,1});
    Nh=zeros(Ln,Cn);
    Kh=zeros(Ln,Cn);
    Mh=zeros(Lm,Cm);
    Rh=zeros(Ln,Cn);
    Bh=zeros(8,4);
    for k=1:ri
        Nh=Nh+h(k)*N{1,k};
        Mh=Mh+h(k)*M{1,k};
        Rh=Rh+h(k)*R{1,k};
        Bh=Bh+h(k)*B{1,k};
    end
%----------------------------------------------
    
    
    dE = [-Nh*Rh' zeros(4);eye(4) zeros(4)]*E+Bh*U;
end

