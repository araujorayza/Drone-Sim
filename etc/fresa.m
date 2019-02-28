function [ accessNum ] = fresa( i, numNL )

    %FRESA Summary of this function goes here
    %   Detailed explanation goes here

    index = i -1;

    num = str2num(dec2bin(index)); 
    Mask = str2num(dec2bin(2^numNL - 1));

    accessNum = num + Mask;

end