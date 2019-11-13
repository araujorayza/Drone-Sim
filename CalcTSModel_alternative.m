function [A,B,NRt,Z] = CalcTSModel_alternative(gamma)
    
    ri=4; % # of local models
    
    Psi=0:0.01:2*pi;
    
    v1=zeros(1,length(Psi));
    v2=zeros(1,length(Psi));
    
    
    for i=1:length(Psi)
        v1(i)=gamma(2)*cos(Psi(i))^2+gamma(4)*sin(Psi(i))^2;
        v2(i)=(gamma(2)-gamma(4))*sin(2*Psi(i))/2;
    end
    
    
    z11=max(v1);
    z10=min(v1);
    z21=max(v2);
    z20=min(v2);
    
    
    NRt=cell(1,4);
    NRt{1,1}=[z11 z21 0 0;z21 gamma(2)+gamma(4)-z11 0 0;0 0 gamma(6) 0;0 0 0 gamma(8)];
    NRt{1,2}=[z11 z20 0 0;z20 gamma(2)+gamma(4)-z11 0 0;0 0 gamma(6) 0;0 0 0 gamma(8)];
    NRt{1,3}=[z10 z21 0 0;z21 gamma(2)+gamma(4)-z10 0 0;0 0 gamma(6) 0;0 0 0 gamma(8)];
    NRt{1,4}=[z10 z20 0 0;z20 gamma(2)+gamma(4)-z10 0 0;0 0 gamma(6) 0;0 0 0 gamma(8)];
    
    A=cell(1,4);
    B=cell(1,4);
    n=size(NRt{1,1},1);
    ri=size(A,2);
    
    for i=1:ri
        A{1,i}=[-NRt{1,i} zeros(n);
            eye(n) zeros(n)];
        B{1,i}=[eye(4);
            zeros(n)];
    end

    Z=[z10 z11;
       z20 z21];
end

