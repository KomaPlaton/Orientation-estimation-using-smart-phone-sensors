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

%% Task 2

% clear;clc;
% close all;
% load meas.mat;
% 
% % Find NaN values and delete from the collected data set
% meas_acc = meas.acc(:,~any(isnan(meas.acc),1));
% meas_gyr = meas.gyr(:,~any(isnan(meas.gyr),1));
% meas_mag = meas.mag(:,~any(isnan(meas.mag),1));
% 
% % Select data in the middle segment to eliminate human interference
% meas_acc = meas_acc(:,400:(length(meas_acc)-1000));
% meas_gyr = meas_gyr(:,400:(length(meas_gyr)-1000));
% meas_mag = meas_mag(:,400:(length(meas_mag)-1000));
% 
% % Compute mean and covariances for the data collected
% acc_mean = mean(meas_acc,2);
% gyr_mean = mean(meas_gyr,2);
% mag_mean = mean(meas_mag,2);
% 
% acc_cov_xaxis = cov(meas_acc(1,:));
% acc_cov_yaxis = cov(meas_acc(2,:));
% acc_cov_zaxis = cov(meas_acc(3,:));
% gyr_cov_xaxis = cov(meas_gyr(1,:));
% gyr_cov_yaxis = cov(meas_gyr(2,:));
% gyr_cov_zaxis = cov(meas_gyr(3,:));
% mag_cov_xaxis = cov(meas_mag(1,:));
% mag_cov_yaxis = cov(meas_mag(2,:));
% mag_cov_zaxis = cov(meas_mag(3,:));
% 
% % Plot histograms of measurements for some sensors and axes
% figure('Position',[300 300 800 600]);
% subplot(3,1,1);
% histogram(meas_acc(1,:),'Normalization','pdf');
% hold on;
% plot_normal_distribution(acc_mean(1), acc_cov_xaxis);
% legend('Accelerometer X axis', 'PDF with calculated mean and covariance');
% grid on;
% subplot(3,1,2);
% histogram(meas_acc(2,:),'Normalization','pdf');
% hold on;
% plot_normal_distribution(acc_mean(2), acc_cov_yaxis);
% legend('Accelerometer Y axis', 'PDF with calculated mean and covariance');
% grid on;
% subplot(3,1,3);
% histogram(meas_acc(3,:),'Normalization','pdf');
% hold on;
% plot_normal_distribution(acc_mean(3), acc_cov_zaxis);
% legend('Accelerometer Z axis', 'PDF with calculated mean and covariance');
% grid on;
% print('Task2_histogram_acc.eps','-depsc');
% 
% figure('Position',[300 300 800 600]);
% subplot(3,1,1);
% histogram(meas_gyr(1,:),'Normalization','pdf');
% hold on;
% plot_normal_distribution(gyr_mean(1), gyr_cov_xaxis);
% legend('Gyroscope X axis', 'PDF with calculated mean and covariance');
% grid on;
% subplot(3,1,2);
% histogram(meas_gyr(2,:),'Normalization','pdf');
% hold on;
% plot_normal_distribution(gyr_mean(2), gyr_cov_yaxis);
% legend('Gyroscope Y axis', 'PDF with calculated mean and covariance');
% grid on;
% subplot(3,1,3);
% histogram(meas_gyr(3,:),'Normalization','pdf');
% hold on;
% plot_normal_distribution(gyr_mean(3), gyr_cov_zaxis);
% legend('Gyroscope Z axis', 'PDF with calculated mean and covariance');
% grid on;
% print('Task2_histogram_gyr.eps','-depsc');
% 
% figure('Position',[300 300 800 600]);
% subplot(3,1,1);
% histogram(meas_mag(1,:),'Normalization','pdf');
% hold on;
% plot_normal_distribution(mag_mean(1), mag_cov_xaxis);
% legend('Magnetometer X axis', 'PDF with calculated mean and covariance');
% grid on;
% subplot(3,1,2);
% histogram(meas_mag(2,:),'Normalization','pdf');
% hold on;
% plot_normal_distribution(mag_mean(2), mag_cov_yaxis);
% legend('Magnetometer Y axis', 'PDF with calculated mean and covariance');
% grid on;
% subplot(3,1,3);
% histogram(meas_mag(3,:),'Normalization','pdf');
% hold on;
% plot_normal_distribution(mag_mean(3), mag_cov_zaxis);
% legend('Magnetometer Z axis', 'PDF with calculated mean and covariance');
% grid on;
% print('Task2_histogram_mag.eps','-depsc');
% 
% % Plot signals over time for the Accelerometer
% figure('Position',[300 300 800 600]);
% plot(meas.t,meas.acc,'LineWidth',2);
% grid on
% title('acc')
% xlabel('time(s)')
% ylabel('m/s^2')
% legend('x','y','z')
% print('acc.eps','-depsc');
% % Plot signals over time for the Gyroscope
% figure('Position',[300 300 800 600]);
% plot(meas.t,meas.gyr,'LineWidth',2);
% grid on
% title('gyro')
% xlabel('time(s)')
% ylabel('rad/s')
% legend('x','y','z')
% print('gyro.eps','-depsc');
% % Plot signals over time for the Magnetometer
% figure('Position',[300 300 800 600]);
% plot(meas.t,meas.mag,'LineWidth',2);
% grid on
% title('magnetic')
% xlabel('time(s)')
% ylabel('\muT')
% legend('x','y','z')
% print('mag.eps','-depsc');

%% Task 5&6&7&8&9&10&11&12
clear;clc;
close all;
% initialize enviroment
startup;
% check IP 
showIP;
% start reciving data from App streaming
[xhat, meas] = UpdatedfilterTemplate();
