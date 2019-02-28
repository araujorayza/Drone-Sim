function [ h ] = Fo(indexVec, M, numNL)

    h = 1;

    for i = 1:numNL
        h = h*accessMatrix(indexVec,M,i);
    end

end

function [ Mij ] = accessMatrix(indexVec, M, i)

    j = int2str(indexVec)-'0';

    Mij = M(i,j(i));
    
end