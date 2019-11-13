function [A,B,M,N,R,Z] = CalcTSModel(gamma)
    
    ri=4; % # of local models
    z10 = -1; 
    z11 =  1;  
    z20 = -1; 
    z21 =  1;  
    
    g1=gamma(1); g2=gamma(2); g3=gamma(3); g4=gamma(4);
    g5=gamma(5); g6=gamma(6); g7=gamma(7); g8=gamma(8);
    M=cell(1,ri);
    N=cell(1,ri);
    R=cell(1,ri);
    
    %      M = [gamma(1)*cos(psi), -gamma(3)*sin(psi),      0,        0;
    %           gamma(1)*sin(psi),  gamma(3)*cos(psi),      0,        0;
    %                 0,                  0,              gamma(5),   0;
    %                 0,                  0,                0,    gamma(7)];
    M{1,1}=[g1*z10 -g3*z20   0   0
        g1*z20  g3*z10   0   0
        0     0    g5   0
        0     0     0   g7];
    M{1,2}=[g1*z10 -g3*z21   0   0
        g1*z21  g3*z10   0   0
        0     0    g5   0
        0     0     0   g7];
    M{1,3}=[g1*z11 -g3*z20   0   0
        g1*z20  g3*z11   0   0
        0     0    g5   0
        0     0     0   g7];
    M{1,4}=[g1*z11 -g3*z21   0   0
        g1*z21  g3*z11   0   0
        0     0    g5   0
        0     0     0   g7];
%     celldisp(M)
    % ********************************
    %     N = [ g2*cos(psi) -g4*sin(psi)   0   0
    %           g2*sin(psi)  g4*cos(psi)   0   0
    %                 0               0   g6   0
    %                 0               0    0   g8]
    N{1,1}=[g2*z10 -g4*z20  0  0
        g2*z20  g4*z10  0  0
        0       0   g6  0
        0       0    0 g8];
    N{1,2}=[g2*z10 -g4*z21  0  0
        g2*z21  g4*z10  0  0
        0       0   g6  0
        0       0    0 g8];
    N{1,3}=[g2*z11 -g4*z20  0  0
        g2*z20  g4*z11  0  0
        0       0   g6  0
        0       0    0 g8];
    N{1,4}=[g2*z11 -g4*z21  0  0
        g2*z21  g4*z11  0  0
        0       0   g6  0
        0       0    0 g8];
%     celldisp(N)
    % ********************************
    %     R = [ cos(psi),    -sin(psi),      0,    0;
    %           sin(psi),     cos(psi),      0,    0;
    %                 0,          0,         1,    0;
    %                 0,          0,         0,    1];
    R{1,1}=[z10 -z20  0  0
        z20  z10  0  0
        0    0   1  0
        0    0   0  1];
    R{1,2}=[z10 -z21  0  0
        z21  z10  0  0
        0    0   1  0
        0    0   0  1];
    R{1,3}=[z11 -z20  0  0
        z20  z11  0  0
        0    0   1  0
        0    0   0  1];
    R{1,4}=[z11 -z21  0  0
        z21  z11  0  0
        0    0   1  0
        0    0   0  1];
%     celldisp(R)


    A=cell(1,ri);
    B=cell(1,ri);
    n=size(N{ri},1);

    for i=1:ri
        A{1,i}=[-N{i}*R{i}' zeros(n);
            eye(n) zeros(n)];
        B{1,i}=[eye(4);
            zeros(n)];
    end
    Z=[z10 z11;
       z20 z21];
end

