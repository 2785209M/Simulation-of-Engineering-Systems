clear; clc; close all;

% Parameters
Bsm = 0.03;       % Damping coefficient Nm/rad/s
Bsf = 1.5;        % Damping coefficient Nm/rad/s
Jm  = 0.003;      % Moment of inertia kgm^2
Jg1 = 0.001;      % Moment of inertia for the gear kgm^2
Jf  = 0.0204;     % Moment of intertia of the forearm and hand kgm^2
g   = 9.81;       % Gravity
Gc  = 0.2;        % Controller gain 
GR  = 1.5;        % Gear ratio
Ke  = 0.35;       % V/(rad/s)
Kf  = 0.5;        % Torque gain
Kg  = 3;          % Gear Compensator gain
Kr  = 1.2;        % Reference Amp gain
Ks  = 1.3;        % Actuator Sensor gain
Kt  = 0.35;       % Nm/A
L   = 0.12;       % Inducatance H
lf  = 0.35;       % Length of the forearm m
mf  = 0.5;        % Mass of the forearm and hand kg
R   = 4;          % Resistance Ohm


ThetaU = deg2rad(7); 
ThetaF0 = deg2rad(7);

% Simulation Parameters
stepsize = 0.001;
comminterval = 0.005;
endtime = 10;

% KI values to sweep
KI_list = [0.1 0.5 1 2 3];
colors = lines(length(KI_list));

% Storage cell arrays
results_t = cell(1,length(KI_list));
results_thetaM = cell(1,length(KI_list));
results_thetaG1 = cell(1,length(KI_list));
results_thetaF = cell(1,length(KI_list));

% Reference Angle
Thetaref = deg2rad(55);

for k = 1:length(KI_list)

    Ki = KI_list(k);  % current KI
    I = 0;            % Integral state
    
    % Initial states
    x = zeros(7,1);
    x(6) = ThetaF0;

    tout = []; xout = []; xdout = [];
    integral_DTheta = 0;

    i=0;
    for time = 0:stepsize:endtime

        e = Thetaref - (x(2) * Ks); % Calculate error
        I = I + (e * stepsize); % Sum of errors over time (Integral State)
        KI = Ki * I; % Define Integral Term
        Ve = Gc * (e + KI); % Calculate controller Voltage with integral term
        Va = Ve * Kg; % Actuator Input Voltage

        % Update state derivatives
        xdot = robot_arm(x, Va);

        % Integration
        x = rk4int(@robot_arm, x, stepsize, Va);

        % Store every comm interval
        if mod(time, comminterval) == 0
            i = i + 1;
            tout(i) = time;
            xout(i,:) = x';
        end
    end

    % Store results for plotting
    results_t{k} = tout;
    results_thetaM{k} = xout(:,2);
    results_thetaG1{k} = xout(:,4);
    results_thetaF{k} = xout(:,6);

end

% PLOTTING
% -------- Motor Angle θ_M -------
figure;
subplot(3,1,1);
hold on; grid on;
for k = 1:length(KI_list)
    plot(results_t{k}, results_thetaM{k}, 'LineWidth', 1.5, 'Color', colors(k,:));
end
title('Motor Angle \theta_M');
xlabel('Time (s)'); ylabel('\theta_M (rad)');
legend("KI="+string(KI_list));
hold off;

% -------- Gear Angle θ_G1 -------
subplot(3,1,2);
hold on; grid on;
for k = 1:length(KI_list)
    plot(results_t{k}, results_thetaG1{k}, 'LineWidth', 1.5, 'Color', colors(k,:));
end
title('Gear Angle \theta_{G1}');
xlabel('Time (s)'); ylabel('\theta_{G1} (rad)');
legend("KI="+string(KI_list));
hold off;

% -------- Forearm Angle θ_F -------
subplot(3,1,3);
hold on; grid on;
for k = 1:length(KI_list)
    plot(results_t{k}, results_thetaF{k}, 'LineWidth', 1.5, 'Color', colors(k,:));
end
title('Forearm Angle \theta_F');
xlabel('Time (s)'); ylabel('\theta_F (rad)');
legend("KI="+string(KI_list));
hold off;