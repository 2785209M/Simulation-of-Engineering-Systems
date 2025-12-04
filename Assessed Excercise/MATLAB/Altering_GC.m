clear; clc; close all;

global Bsm Bsf Jm Jg1 Jf g GR Ke Kf Kt L lf mf R ThetaU

% Parameters
Gc  = [3.5, 2, 1, 0.5, 0.2, 0.1];   % Controller gains

Bsm = 0.03;     
Bsf = 1.5;     
Jm  = 0.003;   
Jg1 = 0.001;  
Jf  = 0.0204; 
g   = 9.81;   
GR  = 1.5;    
Ke  = 0.35;   
Kf  = 0.5;    
Kg  = 3;      
Kr  = 1.2;    
Ks  = 1.3;    
Kt  = 0.35;    
L   = 0.12;    
lf  = 0.35;    
mf  = 0.5;     
R   = 4;       

ThetaU = deg2rad(7);
ThetaF = deg2rad(7);
Thetaref = deg2rad(55);

stepsize = 0.001;
comminterval = 0.005;
endtime = 10;

% Storage for each Gc run
tout_all = {};
xout_all = {};

% SIMULATION LOOP FOR EACH Gc

for k = 1:length(Gc)

    G = Gc(k);
    fprintf("Simulating for Gc = %.2f...\n", G);

    % Initial conditions
    x = zeros(7,1);
    x(6) = ThetaF;  
    integral_DTheta = 0;

    tout = [];
    xout = [];
    i = 0;

    for time = 0:stepsize:endtime

        deltatheta = (Thetaref * Kr) - (x(2) * Ks);
        integral_DTheta = integral_DTheta + stepsize * deltatheta;

        Ve = G * deltatheta;
        Va = Ve * Kg;

        % Derivatives
        xdot = robot_arm(x, Va);

        % RK4 Integration
        x = rk4int(@robot_arm, x, stepsize, Va);

        % Store data
        if mod(time, comminterval) == 0
            i = i + 1;
            tout(i) = time;
            xout(i,:) = x';  
        end
    end

    % Save this run
    tout_all{k} = tout;
    xout_all{k} = xout;

end

fprintf("\nSimulation completed for all Gc values.\n");

figure;

%  Plot θ_M (x(:,2))
subplot(3,1,1);
hold on; grid on;
for k = 1:length(Gc)
    plot(tout_all{k}, xout_all{k}(:,2), 'DisplayName', sprintf("Gc = %.2f", Gc(k)));
end
xlabel("Time (s)");
ylabel("\theta_M (rad)");
title("Motor Angle for Different Gc Values");
legend show;


%  Plot θ_G1 (x(:,4))
subplot(3,1,2);
hold on; grid on;
for k = 1:length(Gc)
    plot(tout_all{k}, xout_all{k}(:,4), 'DisplayName', sprintf("Gc = %.2f", Gc(k)));
end
xlabel("Time (s)");
ylabel("\theta_{G1} (rad)");
title("Gear 1 Angle for Different Gc Values");
legend show;


%  Plot θ_F (forearm) (x(:,6))
subplot(3,1,3);
hold on; grid on;
for k = 1:length(Gc)
    plot(tout_all{k}, xout_all{k}(:,6), 'DisplayName', sprintf("Gc = %.2f", Gc(k)));
end
xlabel("Time (s)");
ylabel("\theta_F (rad)");
title("Forearm Angle for Different Gc Values");
legend show;



%  System Dynamics Function
function xdot = robot_arm(x, Va)
    global Bsm Bsf Jm Jg1 Jf g GR Ke Kf Kt L lf mf R ThetaU

    Tf = (GR*Kf*x(4));

    xdot = zeros(7,1);
    xdot(1) = -(R*x(1))/L - (Ke/L)*x(3) + Va/L;
    xdot(2) = x(3);
    xdot(3) = (Kt*x(1) - Bsm*(x(3)-x(5))) / Jm;
    xdot(4) = x(5);
    xdot(5) = (Bsm*(x(3)-x(5))) / Jg1;
    xdot(6) = x(7);
    xdot(7) = (Tf/Jf) - (Bsf/Jf)*x(7) - ((mf*lf)/(2*Jf))*g*sin(ThetaU+x(6));
end


%  RK4 Integrator
function xnew = rk4int(fhandle, xcur, dt, Va_local)
    k1 = fhandle(xcur, Va_local);
    k2 = fhandle(xcur + 0.5*dt*k1, Va_local);
    k3 = fhandle(xcur + 0.5*dt*k2, Va_local);
    k4 = fhandle(xcur + dt*k3, Va_local);
    xnew = xcur + dt*(k1 + 2*k2 + 2*k3 + k4)/6;
end