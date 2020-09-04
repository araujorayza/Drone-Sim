function [A,B,h] = CalcTSModel(gamma)
    psi = sym('psi','real');
    nl = [cos(psi);
        sin(psi)];

    z10 = -1; %z1 min
    z11 =  1; %z1 max
    z20 = -1; %z2 min
    z21 =  1; %z2 max

    ri = 2^length(nl); % # of local models based on nonlinearities

    % Z(z_i,min_or_max)
    Z=[z10 z11;
        z20 z21];

    M=cell(1,ri);
    N=cell(1,ri);
    R=cell(1,ri);

    z = 0*nl;
    for i = 1:ri
        j = AccessMask(ri,i);

        for k=1:length(z)
            z(k) = Z(k,j(k));
        end

        M{i} = [ gamma(1)*z(1), -gamma(3)*z(2),      0,            0;
            gamma(1)*z(2),  gamma(3)*z(1),      0,            0;
            0,                  0,           gamma(5),        0;
            0,                  0,              0,     gamma(7)];

        N{i} = [ gamma(2)*z(1), -gamma(4)*z(2),      0,            0;
            gamma(2)*z(2),  gamma(4)*z(1),      0,            0;
            0,                  0,         gamma(6),       0;
            0,                  0,            0,    gamma(8)];


        R{i} = [ z(1),    -z(2),      0,    0;
            z(2),     z(1),      0,    0;
            0,            0,      1,    0;
            0,            0,      0,    1];
    end

    % Definition of the membership functions
    h=MembFunc(nl,Z);

    % Model matrices
    A=cell(ri,ri);
    B=cell(1,ri);

    for i=1:ri
        for j=1:ri
            A{i,j}=double([-N{i}*R{j}' 0*eye(4);
                eye(4) zeros(4)]);
        end
        B{i}=double([eye(4);
            zeros(4)]);
    end
end

%% Auxiliar Functions
function  j = AccessMask(numLines,i)
    %we use a binary count to access all possible
    %combinations of the membership functions/nonlinearity values
    % i.e. 0 0, ----->  1 1
    %      0 1,         1 2
    %      1 0,         2 1
    %      1 1          2 2
    
    %The mask basically sums 1 to all the bits
    %because the first element of a matlab matrix
    %is 1, not 0.
    Mask = str2num(dec2bin(numLines - 1)); 
    
    % i indicates which line we want to access 
    num = str2num(dec2bin(i-1));    
    
    %each h component/local model 
    % corresponds to the (i-1)th binary
    % count, then we add the mask to use
    % the result as indexes for matrix
    % column access
    accessNum = num + Mask;
    
    %This line separates the masked access number 'bits' 
    % putting each in a vector column
	j = int2str(accessNum)-'0';     

end 

function [h] = MembFunc(z,Z)
    % Calculates the defuzzyfication weights for fuzzy controller
    % The function returns a vector with all h(z). Each of them is the product
    % of the membership functions associated with that local model.
    % Ex.:                                    M(nonLin)_(which_func)
    %                   A_1 = [0 z1_2;      h1 = M1_2 * M2_2;
    %                          1 z2_2];
    %
    %                   A_2 = [0 z1_2;      h2 = M1_2 * M2_1;
    %                          1 z2_1];
    %
    MIN=1;MAX=2;
    H = sym('H',[length(z) 2]);
    
    for i=1:length(z)
        H(i,1) = (Z(i,MAX)- z(i))/(Z(i,MAX)-Z(i,MIN)); %it's 1 when z = min
        H(i,2) =  1 - H(i,1);
    end
    
    numLines = 2^(size(H,1)); %number of h functions is the # of possible
    %combinations between the nonlinearities
   
    h=ones(numLines,1);
    h=cast(h,class(H));

    for i = 1:numLines
        j = AccessMask(numLines,i);
        h(i) = calculate_h(H,j);
    end                         

end

function [h] = calculate_h(H,j)
    h=1;
    for i=1:size(H,1)
        h=h*H(i,j(i));
    end
end


