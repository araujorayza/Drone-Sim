function [EstmState] = KalmanFilter(MeasState)
%KALMANFILTER Summary of this function goes here
%   Detailed explanation goes here
persistent P;

NextState=A_kalman;
EstmState=MeasState;
end

