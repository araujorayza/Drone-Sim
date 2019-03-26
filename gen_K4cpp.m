function Knew = gen_K4cpp(K)
    
    Tamlin_K = length(K)*size(K{1},1);
    Tamcol_K = size(K{1},2);
    
    Knew=zeros(Tamlin_K,Tamcol_K);
    
    for i=1:length(K)
        Knew((i-1)*length(K)+1:i*length(K),:)=K{i};
    end
%     formatSTR='%6.4f, ';
%     for i=1:size(Knew,2)-1;
%         formatSTR=strcat(formatSTR,'%6.4f, ');
%     end
%     formatSTR=strcat(formatSTR,'\n')
    Knew
    fid=fopen('K_cpp.txt','w');
%     Ktest=[1 2 3; 4 5 6; 7 8 9];
    fprintf(fid,'%6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f,\n',Knew');
%     fprintf(fid,'%6.4f, %6.4f, %6.4f \n',Ktest');
    fclose(fid);
   
end

