function V=function_total_matrix (p)

% Local models
ri=2^p;

% Total of columns possibilities
m2=2^ri;

% Number of columns of Total matrix
co=factorial(2^p)/(factorial(2^p/2)^2);


th=0:1:m2-1;

Vaux1=cell(1,m2);

for i=1:length(th)
    Vaux1{1,i}=de2bi([th(i)],ri);
end

Vaux2=zeros(ri,m2);
for j=1:m2
    for i=1:ri
        Vaux2(i,j)=(-1)^Vaux1{1,j}(i);
    end
end

% Total matrix
V=[];

for j=1:m2
        if sum(Vaux2(:,j))==0
            V=[V, Vaux2(:,j)];
       end
end


end

