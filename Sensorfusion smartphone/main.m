% clear;clc;
% close all;
% % initialize enviroment
% startup;
% % check IP 
% showIP;
% % start reciving data from App streaming
% [xhat, meas] = filterTemplate();
% save meas;
% save xhat;

% %% Task 2
% clear;clc;
% close all;
% load meas.mat;
% 
% %%histogram
% meas_acc = meas.acc(:,~any(isnan(meas.acc),1));
% meas_gyr = meas.gyr(:,~any(isnan(meas.gyr),1));
% meas_mag = meas.mag(:,~any(isnan(meas.mag),1));
% 
% meas_acc = meas_acc(:,400:(length(meas_acc)-1000));
% meas_gyr = meas_gyr(:,400:(length(meas_gyr)-1000));
% meas_mag = meas_mag(:,400:(length(meas_mag)-1000));
% 
% figure('Position',[300 300 600 400]);
% histogram(meas_acc(1,:),'Normalization','pdf');
% figure('Position',[300 300 600 400]);
% histogram(meas_gyr(1,:),'Normalization','pdf')
% figure('Position',[300 300 600 400]);
% histogram(meas_gyr(3,:),'Normalization','pdf')
% figure('Position',[300 300 600 400]);
% histogram(meas_mag(1,:),'Normalization','pdf')
% 
% %% plot
% figure('Position',[300 300 600 400]);
% plot(meas.t,meas.acc,'LineWidth',2);
% hold on
% grid on
% title('acc')
% xlabel('time(s)')
% ylabel('m/s^2')
% legend('x','y','z')
% print('acc.eps','-depsc');
% 
% figure('Position',[300 300 600 400]);
% plot(meas.t,meas.gyr,'LineWidth',2);
% hold on
% grid on
% title('gyro')
% xlabel('time(s)')
% ylabel('rad/s')
% legend('x','y','z')
% print('gyro.eps','-depsc');
% 
% figure('Position',[300 300 600 400]);
% plot(meas.t,meas.mag,'LineWidth',2);
% hold on
% grid on
% title('magnetic')
% xlabel('time(s)')
% ylabel('\muT')
% print('mag.eps','-depsc');
% 
% mean(meas_acc,2);
% mean(meas_gyr,2);
% mean(meas_mag,2);
% 
% cov(meas_acc(1,:));
% cov(meas_acc(2,:));
% cov(meas_acc(3,:));
% cov(meas_gyr(1,:));
% cov(meas_gyr(2,:));
% cov(meas_gyr(3,:));
% cov(meas_mag(1,:));
% cov(meas_mag(2,:));
% cov(meas_mag(3,:));

%% Task 5&6&7&8&9
clear;clc;
close all;
% initialize enviroment
startup;
% check IP 
showIP;
% start reciving data from App streaming
[xhat, meas] = UpdatedfilterTemplate();
