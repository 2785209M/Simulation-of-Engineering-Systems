clear; clc; close all;
global Bsm Bsf Jm Jg1 Jf g GR Ke Kf Kt L lf mf R ThetaU
% Parameters
Bsm = 0.03;     % Damping coefficient Nm/rad/s
Bsf = 1.5;     % Damping coefficient Nm/rad/s
Jm  = 0.003;    % Moment of inertia kgm^2
Jg1 = 0.001;    % Moment of inertia for the gear kgm^2
Jf  = 0.0204;    % Moment of intertia of the forearm and hand kgm^2
g   = 9.81;     % Gravity
Gc  = 3.5;      % Controller gain 
GR  = 1.5;      % Gear ratio
Ke  = 0.35;     % V/(rad/s)
Kf  = 0.5;      % Torque gain
Kg  = 3;        % Gear Compensator gain
Kr  = 1.2;      % Reference Amp gain
Ks  = 1.3;      % Actuator Sensor gain
Kt  = 0.35;      % Nm/A
L   = 0.12;      % Inducatance H
lf  = 0.35;      % Length of the forearm m
mf  = 0.5;       % Mass of the forearm and hand kg
R   = 4;         % Resistance Ohm

ThetaU = deg2rad(7); % Upper Arm Angle
ThetaF = deg2rad(7); % Forearm Angle
Thetaref = deg2rad(55); % Reference angle

x = zeros(7,1); % Initialize array for x
xdot = zeros(7,1); % Initialize array for xdot
x(6) = ThetaF;   % Angle

% Simulation Parameters
stepsize = 0.001;
comminterval = 0.005;
endtime = 10;

% Initialize output arrays
tout  = [];
xout  = [];
xdout = [];

i=0;
for time = 0:stepsize:endtime

    % Derivatives
    deltatheta = (Thetaref * Kr) - (x(2) * Ks);
    Ve = (Gc * deltatheta);
    Va = (Ve * Kg);

    % Update state derivatives
    xdot = robot_arm(x, Va);

    % Integration
    x = rk4int(@robot_arm, x, stepsize, Va);

    % Store time state and state derivative every communication interval
    if mod(time,comminterval) == 0
        i = i + 1;
        tout(i)  = time;
        xout(i,:)  = x';      % Transpose to 1x7
        xdout(i,:) = xdot';   % Transpose to 1x7
    end
end

figure;

% Define shared Y-limits for related variable groups

% Motor current
ylim_i = [min(xout(:,1)) max(xout(:,1))];

% Motor and Gear angles combined
all_angles = [xout(:,2); xout(:,4)];
ylim_theta = [min(all_angles) max(all_angles)];

% Motor and Gear angular velocities combined
all_omegas = [xout(:,3); xout(:,5)];
ylim_omega = [min(all_omegas) max(all_omegas)];

% Forearm angle and velocity separate (different scale)
ylim_thetaF = [min(xout(:,6)) max(xout(:,6))];
ylim_omegaF = [min(xout(:,7)) max(xout(:,7))];


% 1. Motor Current
subplot(4,2,[1 2]);
plot(tout, xout(:,1), 'LineWidth', 1.3);
xlabel('Time (s)');
ylabel('i (A)');
title('Motor Current');
grid on;
ylim(ylim_i);

% 2. Motor Angle
subplot(4,2,3);
plot(tout, xout(:,2), 'LineWidth', 1.3);
xlabel('Time (s)');
ylabel('\theta_M (rad)');
title('Motor Angle');
grid on;
ylim(ylim_theta);

% 3. Motor Angular Velocity
subplot(4,2,4);
plot(tout, xout(:,3), 'LineWidth', 1.3);
xlabel('Time (s)');
ylabel('\omega_M (rad/s)');
title('Motor Angular Velocity');
grid on;
ylim(ylim_omega);

% 4. Gear Angle
subplot(4,2,5);
plot(tout, xout(:,4), 'LineWidth', 1.3);
xlabel('Time (s)');
ylabel('\theta_G1 (rad)');
title('Gear Angle');
grid on;
ylim(ylim_theta);

% 5. Gear Angular Velocity
subplot(4,2,6);
plot(tout, xout(:,5), 'LineWidth', 1.3);
xlabel('Time (s)');
ylabel('\omega_G1 (rad/s)');
title('Gear Angular Velocity');
grid on;
ylim(ylim_omega);

% 6. Forearm Angle
subplot(4,2,7);
plot(tout, xout(:,6), 'LineWidth', 1.3);
xlabel('Time (s)');
ylabel('\theta_F (rad)');
title('Forearm Angle');
grid on;
ylim(ylim_thetaF);

% 7. Forearm Angular Velocity
subplot(4,2,8);
plot(tout, xout(:,7), 'LineWidth', 1.3);
xlabel('Time (s)');
ylabel('\omega_F (rad/s)');
title('Forearm Angular Velocity');
grid on;
ylim(ylim_omegaF);