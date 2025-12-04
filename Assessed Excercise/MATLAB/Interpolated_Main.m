clear; clc; close all;

global Bsm Bsf Jm Jg1 Jf g GR Ke Kf Kt L lf mf R
% Parameters
Bsm = 0.03;     % Damping coefficient Nm/rad/s
Bsf = 1.5;     % Damping coefficient Nm/rad/s
Jm  = 0.003;    % Moment of inertia kgm^2
Jg1 = 0.001;    % Moment of inertia for the gear kgm^2
Jf  = 0.0204;    % Moment of intertia of the forearm and hand kgm^2
g   = 9.81;     % Gravity
Gc  = 0.2;      % Controller gain 
GR  = 1.5;      % Gear ratio
Ke  = 0.35;     % V/(rad/s)
Kf  = 0.5;      % Torque gain
Kg  = 3;      % Gear Compensator gain
Kr  = 1.2;      % Reference Amp gain
Ks  = 1.3;      % Actuator Sensor gain
Kt  = 0.35;      % Nm/A
L   = 0.12;      % Inducatance H
lf  = 0.35;      % Length of the forearm m
mf  = 0.5;       % Mass of the forearm and hand kg
R   = 4;         % Resistance Ohm

ThetaF = deg2rad(7); % Upper Arm Angle
Thetaref = deg2rad(55); % Reference angle

x = zeros(7,1); % Initialize array for x
xdot = zeros(7,1); % Initialize array for xdot
x(6) = ThetaF;   % Angle

% Simulation Parameters
stepsize = 0.001;
comminterval = 0.005;
endtime = 15;

% Initialize output arrays
tout  = [];
xout  = [];
xdout = [];

% time and thetaU values
xvals = [0 1.5 5.5 15 21 29];
yvals = deg2rad([3 10.5 25.5 33 45 50]);
% Calculate Interpolation Coefficients
[table, coeffs] = Calculate_Divided_Differences(xvals,yvals);
thetaUvals = Polynomial_Evaluation(xvals, stepsize, coeffs);

Ki = 0.5;
I=0;
i=0;
for time = 0:stepsize:endtime

    e = Thetaref - (x(2) * Ks); % Calculate error
    I = I + e*stepsize; % Sum of errors over time (Integral State)
    KI = Ki * I; % Define Integral Term
    Ve = Gc * (e + KI); % Calculate controller Voltage with integral term
    Va = Ve * Kg; % Actuator Input Voltage

    % Calculate Realtime thetaU
    thetaU = Interpolation_Realtime(time, xvals, coeffs);

    % Update state derivatives
    xdot = robot_arm(x, Va, thetaU);

    % Integration
    x = rk4int(@robot_arm, x, stepsize, Va, thetaU);

    % Store time state and state derivative every communication interval
    if abs(rem(time, comminterval)) < 1e-10
        i = i + 1;
        tout(i)  = time;
        xout(i,:)  = x';      % Transpose to 1x7
        xdout(i,:) = xdot';   % Transpose to 1x7
    end

end

figure;

% 1. Motor Current
subplot(3,3,[1,2,3]);
plot(tout, xout(:,1), 'LineWidth', 1.3);
xlabel('Time (s)');
ylabel('i (A)');
title('Motor Current');
grid on;
ylim([-0.1 0.15]);

% 2. Motor Angle
subplot(3,3,4);
plot(tout, xout(:,2), 'LineWidth', 1.3);
xlabel('Time (s)');
ylabel('\theta_M (rad)');
title('Motor Angle');
grid on;
ylim([0 1]);

% 3. Motor Angular Velocity
subplot(3,3,5);
plot(tout, xout(:,3), 'LineWidth', 1.3);
xlabel('Time (s)');
ylabel('\omega_M (rad/s)');
title('Motor Angular Velocity');
grid on;
ylim([-0.5 1.5]);

% 4. Gear Angle
subplot(3,3,6);
plot(tout, xout(:,4), 'LineWidth', 1.3);
xlabel('Time (s)');
ylabel('\theta_G1 (rad)');
title('Gear Angle');
grid on;
ylim([0 1]);

% 5. Gear Angular Velocity
subplot(3,3,7);
plot(tout, xout(:,5), 'LineWidth', 1.3);
xlabel('Time (s)');
ylabel('\omega_G1 (rad/s)');
title('Gear Angular Velocity');
grid on;
ylim([-0.5 1.5]);

% 6. Forearm Angle
subplot(3,3,8);
plot(tout, xout(:,6), 'LineWidth', 1.3);
xlabel('Time (s)');
ylabel('\theta_F (rad)');
title('Forearm Angle');
grid on;
ylim([0 1]);

% 7. Forearm Angular Velocity
subplot(3,3,9);
plot(tout, xout(:,7), 'LineWidth', 1.3);
xlabel('Time (s)');
ylabel('\omega_F (rad/s)');
title('Forearm Angular Velocity');
grid on;
ylim([-0.5 1.5]);