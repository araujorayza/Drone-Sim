function [EstmState] = KalmanFilter(MeasState,curTime)
%% KALMANFILTER Summary of this function goes here
%   Detailed explanation goes here
    EstmState=MeasState;
    persistent prevTime;
    persistent P;
    persistent x;
    
    %% check if the variables are empty (first function call)
    if isempty(prevTime)
        prevTime = 0;
    end
    
    if isempty(P)
        P = zeros(12);
    end
    
    if isempty(x)
        x = zeros(12,1);
    end
    %% State transition Matrix
    A_kalman            = eye(12);
        dt              = curTime - prevTime;
    A_kalman(1:4,9:12)  = dt*eye(4);
    A_kalman(5:8,1:4)   = dt*eye(4);
    %Measurement matrix
    H_kalman = [zeros(4),eye(4),zeros(4)]; %measures only the position

    %Kalman matrices
    Q_kalman = 0.01*eye(12);
    R_kalman = 0.1*eye(4); %if zero, obs is perfect
    
     
    %Prediction
    xp 	= A_kalman*x;
    Pp 	= A_kalman*P*A_kalman' + Q_kalman;
    
    %Measurement update
    z = MeasState(5:8);
    
    y = z - H_kalman*xp;
    S = H_kalman*Pp*H_kalman' + R_kalman;
    K 	= Pp*H_kalman'/(S);
    
    
    x 	= xp + K*y;
    P 	= A_kalman*(eye(12) - Pp*H_kalman'*inv(S)*H_kalman)*Pp*A_kalman'; 
                                            % P = Pp - K*H_kalman*Pp 
                                            % is not compulationally stable
    prevTime = curTime;
    
    %Correct the state 
    EstmState(1:4)=x(1:4);
    EstmState(5:8)=x(5:8);    
end

