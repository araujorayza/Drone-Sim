function [h] = calculate_h(H,accessNum)
    j = int2str(accessNum)-'0';
    h=1;
    for i=1:size(H,1)
        h=h*H(i,j(i));
    end
end

