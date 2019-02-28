

M = [1 2;3 4;5 6];
numNL = 3;

numComb = 2^numNL;
h = zeros(numComb,1);

for i = 1:numComb
    indexVec = fresa(i,numNL);
    h(i) = Fo(indexVec,M,numNL);
end