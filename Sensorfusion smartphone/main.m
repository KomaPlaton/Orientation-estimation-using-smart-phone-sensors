clear;clc;
close all;
% initialize enviroment
startup;
% check IP 
showIP;
% start reciving data from App streaming
[xhat, meas] = filterTemplate();
%% plot
figure('Position',[300 300 600 400]);
plot(meas.t,meas.acc,'LineWidth',2);
hold on
grid on
title('acc')
xlabel('time(s)')
ylabel('m/s^2')
legend('x','y','z')
print('acc.eps','-depsc');

figure('Position',[300 300 600 400]);
plot(meas.t,meas.gyr,'LineWidth',2);
hold on
grid on
title('gyro')
xlabel('time(s)')
ylabel('rad/s')
legend('x','y','z')
print('gyro.eps','-depsc');

figure('Position',[300 300 600 400]);
mag_0 = meas.mag;
for i = 1 : length(mag_0)-1
    if isnan(mag_0(:,i+1))
        mag_0(:,i+1) = mag_0(:,i);
    end
end
plot(meas.t,mag_0,'LineWidth',2);
hold on
grid on
title('magnetic')
xlabel('time(s)')
ylabel('\muT')
print('mag.eps','-depsc');