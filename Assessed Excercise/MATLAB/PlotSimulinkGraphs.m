% Extract numeric data from timeseries
iM  = out.X1.Data;
thM = out.X2.Data;
wM  = out.X3.Data;
thG = out.X4.Data;
wG  = out.X5.Data;
thF = out.X6.Data;
wF  = out.X7.Data;

% Compute shared y-limits
angle_min = min([thM; thG; thF]);
angle_max = max([thM; thG; thF]);

vel_min = min([wM; wG; wF]);
vel_max = max([wM; wG; wF]);

figure;

% 1. Motor Current
subplot(4,2,[1 2]);
plot(iM, 'r');
xlabel('Time (s)');
ylabel('i (A)');
title('Motor Current');
ylim([-2 2]);
grid on;

% 2. Motor Angle
subplot(4,2,3);
plot(thM,'r');
xlabel('Time (s)');
ylabel('\theta_M (rad)');
title('Motor Angle');
ylim([angle_min angle_max]);
grid on;

% 3. Motor Angular Velocity
subplot(4,2,4);
plot(wM,'r');
xlabel('Time (s)');
ylabel('\omega_M (rad/s)');
title('Motor Angular Velocity');
ylim([vel_min vel_max]);
grid on;

% 4. Gear Angle
subplot(4,2,5);
plot(thG,'r');
xlabel('Time (s)');
ylabel('\theta_G1 (rad)');
title('Gear Angle');
ylim([angle_min angle_max]);
grid on;

% 5. Gear Angular Velocity
subplot(4,2,6);
plot(wG,'r');
xlabel('Time (s)');
ylabel('\omega_G1 (rad/s)');
title('Gear Angular Velocity');
ylim([vel_min vel_max]);
grid on;

% 6. Forearm Angle
subplot(4,2,7);
thF_norm = thF ./ max(abs(thF));
plot(thF_norm,'r');
xlabel('Time (s)');
ylabel('\theta_F (norm)');
title('Forearm Angle (Normalized)');
ylim([0 1]);
grid on;


% 7. Forearm Angular Velocity
subplot(4,2,8);
plot(wF,'r');
xlabel('Time (s)');
ylabel('\omega_F (rad/s)');
title('Forearm Angular Velocity');
ylim([-0.1 0.6]);
grid on;