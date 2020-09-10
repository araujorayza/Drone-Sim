clc;
clear all;
close all;
syms psi real
gamma = sym('gamma',[8 1]);
assumeAlso(gamma,'real');
global small_k;


M = [ gamma(1)*cos(psi), -gamma(3)*sin(psi),      0,            0;
      gamma(1)*sin(psi),  gamma(3)*cos(psi),      0,            0;
      0,                  0,               gamma(5),            0;
      0,                  0,                      0,    gamma(7)];

N = [ gamma(2)*cos(psi), -gamma(4)*sin(psi),      0,            0;
      gamma(2)*sin(psi),  gamma(4)*cos(psi),      0,            0;
      0,                  0,               gamma(6),            0;
      0,                  0,                      0,    gamma(8)];


R = [ cos(psi),    -sin(psi),      0,    0;
      sin(psi),     cos(psi),      0,    0;
      0,            0,             1,    0;
      0,            0,             0,    1];

