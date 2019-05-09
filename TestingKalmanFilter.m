clear all
t=0:0.25:2*pi;

STATE_real=[sin(t);cos(t);sin(2*t);cos(2*t);...
            sin(3*t);cos(3*t);sin(4*t);cos(4*t)];

noise = wgn(8,length(t),0.1,'dBm');
STATE_meas = STATE_real + noise;

STATE_filter = STATE_meas;

for i = 1:length(t)
    STATE_filter(:,i)=KalmanFilter(STATE_meas(:,i),t(i));
end

plot(t,STATE_real(1:1,:),'r',...
     t,STATE_meas(1:1,:),'b',...
     t,STATE_filter(1:1,:),'k');
