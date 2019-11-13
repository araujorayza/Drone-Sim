function [h] = CalcDefuzzWeights(H)
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
    numLines = 2^(size(H,1)); %number of h functions is the # of possible
                              %combinations between the nonlinearities
    
    Mask = str2num(dec2bin(numLines - 1)); % Mask to acess the matrix H
                              %we use a binary count to access all possible
                              %combinations of the membership functions.
                              % i.e. 0 0, ----->  1 1
                              %      0 1,         1 2
                              %      1 0,         2 1
                              %      1 1          2 2                  
                              %This mask basically sums 1 to all the bits
                              %because the first element of a matlab matrix
                              %is 1, not 0. 
    h=ones(numLines,1); 
    for i = 1:length(h)              
        num = str2num(dec2bin(i-1));    %each h component corresponds to the
        accessNum = num + Mask;         %(i-1)th binary count, then we add 
        h(i) = calculate_h(H,accessNum);% the mask to use the result as
                                        %indexes for matrix column access
    
    end                                 

end

function [h] = calculate_h(H,accessNum)
    j = int2str(accessNum)-'0'; %separates each algarism of accessNum
    h=1;                        %in a column of vector j
    for i=1:size(H,1)
        h=h*H(i,j(i));
    end
end