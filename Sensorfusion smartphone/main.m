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
% % Plot histograms of measurements and signals over time
% % For accelerometer measurements
% figure('Position',[300 300 1200 600]);
% subplot(3,2,1);
% histogram(meas_acc(1,:),'Normalization','pdf');
% hold on;
% plot_normal_distribution(acc_mean(1), acc_cov_xaxis);
% title('Histogram of accelerometer measurements')
% legend('Acc X axis', 'PDF');
% grid on;
% subplot(3,2,2);
% plot(1:length(meas_acc), meas_acc(1,:),'LineWidth',0.5);
% grid on
% title('Accelerometer signals over time')
% xlabel('time(s)')
% ylabel('m/s^2')
% subplot(3,2,3);
% histogram(meas_acc(2,:),'Normalization','pdf');
% hold on;
% plot_normal_distribution(acc_mean(2), acc_cov_yaxis);
% legend('Acc Y axis', 'PDF');
% grid on;
% subplot(3,2,4);
% plot(1:length(meas_acc), meas_acc(2,:),'LineWidth',0.5);
% grid on
% xlabel('time(s)')
% ylabel('m/s^2')
% subplot(3,2,5);
% histogram(meas_acc(3,:),'Normalization','pdf');
% hold on;
% plot_normal_distribution(acc_mean(3), acc_cov_zaxis);
% legend('Acc Z axis', 'PDF');
% grid on;
% subplot(3,2,6);
% plot(1:length(meas_acc), meas_acc(3,:),'LineWidth',0.5);
% grid on
% xlabel('time(s)')
% ylabel('m/s^2')
% print('Task2_histogram_acc_new.eps','-depsc');
% 
% % For gyroscope measurements
% figure('Position',[300 300 1200 600]);
% subplot(3,2,1);
% histogram(meas_gyr(1,:),'Normalization','pdf');
% title('Histogram of gyroscope measurements')
% hold on;
% plot_normal_distribution(gyr_mean(1), gyr_cov_xaxis);
% legend('Gyroscope X axis', 'PDF');
% grid on;
% subplot(3,2,2);
% plot(1:length(meas_gyr) ,meas_gyr(1,:),'LineWidth',0.5);
% title('Gyroscope signals over time')
% grid on
% xlabel('time(s)')
% ylabel('rad/s')
% subplot(3,2,3);
% histogram(meas_gyr(2,:),'Normalization','pdf');
% hold on;
% plot_normal_distribution(gyr_mean(2), gyr_cov_yaxis);
% legend('Gyroscope Y axis', 'PDF');
% grid on;
% subplot(3,2,4);
% plot(1:length(meas_gyr) ,meas_gyr(2,:),'LineWidth',0.5);
% grid on
% xlabel('time(s)')
% ylabel('rad/s')
% subplot(3,2,5);
% histogram(meas_gyr(3,:),'Normalization','pdf');
% hold on;
% plot_normal_distribution(gyr_mean(3), gyr_cov_zaxis);
% legend('Gyroscope Z axis', 'PDF');
% grid on;
% subplot(3,2,6);
% plot(1:length(meas_gyr) ,meas_gyr(3,:),'LineWidth',0.5);
% grid on
% xlabel('time(s)')
% ylabel('rad/s')
% print('Task2_histogram_gyr_new.eps','-depsc');
% 
% % For magnetometer measurements
% figure('Position',[300 300 1200 600]);
% subplot(3,2,1);
% histogram(meas_mag(1,:),'Normalization','pdf');
% title('Histogram of magnetometer measurements')
% hold on;
% plot_normal_distribution(mag_mean(1), mag_cov_xaxis);
% legend('Magnetometer X axis', 'PDF');
% grid on;
% subplot(3,2,2);
% plot(1:length(meas_mag),meas_mag(1,:),'LineWidth',0.5);
% title('Magnetometer signals over time')
% grid on
% xlabel('time(s)')
% ylabel('\muT')
% subplot(3,2,3);
% histogram(meas_mag(2,:),'Normalization','pdf');
% hold on;
% plot_normal_distribution(mag_mean(2), mag_cov_yaxis);
% legend('Magnetometer Y axis', 'PDF');
% grid on;
% subplot(3,2,4);
% plot(1:length(meas_mag),meas_mag(2,:),'LineWidth',0.5);
% grid on
% xlabel('time(s)')
% ylabel('\muT')
% subplot(3,2,5);
% histogram(meas_mag(3,:),'Normalization','pdf');
% hold on;
% plot_normal_distribution(mag_mean(3), mag_cov_zaxis);
% legend('Magnetometer Z axis', 'PDF');
% grid on;
% subplot(3,2,6);
% plot(1:length(meas_mag),meas_mag(3,:),'LineWidth',0.5);
% grid on
% xlabel('time(s)')
% ylabel('\muT')
% print('Task2_histogram_mag.eps','-depsc');

% %% Task 5&6&7&8&9&10&11&11
% clear;clc;
% close all;
% % initialize enviroment
% startup;
% % check IP 
% showIP;
% % start reciving data from App streaming
% [xhat, meas] = UpdatedfilterTemplate();
% %% Plot results vs google (all sensors)
% euler_own = q2euler(xhat.x);
% euler_google = q2euler(meas.orient);
% figure('Position',[300 300 600 400]);
% hold on;
% for i=1:3
%     subplot(3,1,i);
%     hold on
%     plot(euler_own(i,:),'color','red','LineWidth',1);
%     plot(euler_google(i,:),'color','black','LineWidth',1);
%     ylim([-5 5])
%     legend('OWN','Google')
%     if i == 1
%         title('yaw')
%     elseif i == 2
%         title('pitch')
%     else
%         title('roll')
%     end
% end
% %yaw pitch roll

%% Task 12 
clear;clc;
close all;
% initialize enviroment
startup;
% check IP 
showIP;
% start reciving data from App streaming
[xhat, meas] = UpdatedfilterTemplate();
% Plot results vs google (all sensors)
euler_own0 = q2euler(xhat.x);
euler_google0 = q2euler(meas.orient);
figure('Position',[300 300 600 400]);
hold on;
for i=1:3
    subplot(3,1,i);
    hold on
    plot(euler_own0(i,:),'color','red','LineWidth',1);
    plot(euler_google0(i,:),'color','black','LineWidth',1);
    ylim([-5 5])
    legend('OWN','Google')
    if i == 1
        title('yaw')
    elseif i == 2
        title('pitch')
    else
        title('roll')
    end
end
%yaw pitch roll
print('Allsensors.eps','-depsc');

%% Plot results vs google (acc & mag)
[xhat1, meas1] = AMUpdatedfilterTemplate();
euler_own1 = q2euler(xhat1.x);
euler_google1 = q2euler(meas1.orient);
figure('Position',[300 300 600 400]);
hold on;
for i=1:3
    subplot(3,1,i);
    hold on
    plot(euler_own1(i,:),'color','red','LineWidth',1);
    plot(euler_google1(i,:),'color','black','LineWidth',1);
    ylim([-5 5])
    legend('OWN','Google')
    if i == 1
        title('yaw')
    elseif i == 2
        title('pitch')
    else
        title('roll')
    end
end
%yaw pitch roll
print('accandmag.eps','-depsc');
%% Plot results vs google (gyr & acc)
[xhat2, meas2] = GAUpdatedfilterTemplate();
euler_own2 = q2euler(xhat2.x);
euler_google2 = q2euler(meas2.orient);
figure('Position',[300 300 600 400]);
hold on;
for i=1:3
    subplot(3,1,i);
    hold on
    plot(euler_own2(i,:),'color','red','LineWidth',1);
    plot(euler_google2(i,:),'color','black','LineWidth',1);
    ylim([-5 5])
    legend('OWN','Google')
    if i == 1
        title('yaw')
    elseif i == 2
        title('pitch')
    else
        title('roll')
    end
end
%yaw pitch roll
print('gyrandacc.eps','-depsc');
%% Plot results vs google (gyr & mag)
[xhat3, meas3] = GMUpdatedfilterTemplate();
euler_own3 = q2euler(xhat3.x);
euler_google3 = q2euler(meas3.orient);
figure('Position',[300 300 600 400]);
hold on;
for i=1:3
    subplot(3,1,i);
    hold on
    plot(euler_own3(i,:),'color','red','LineWidth',1);
    plot(euler_google3(i,:),'color','black','LineWidth',1);
    ylim([-5 5])
    legend('OWN','Google')
    if i == 1
        title('yaw')
    elseif i == 2
        title('pitch')
    else
        title('roll')
    end
end
%yaw pitch roll
print('gyrandmag.eps','-depsc');