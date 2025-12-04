function xdot = robot_arm(x, Va, ThetaU)
    global Bsm Bsf Jm Jg1 Jf g GR Ke Kf Kt L lf mf R
    Tf = (GR*Kf*x(4));

    xdot = zeros(7,1);
    
    % Motor current
    xdot(1) = -(R*x(1))/L - (Ke/L)*x(3)  + Va/L;  % di/dt
    % Actuator dynamics
    xdot(2) = x(3);
    xdot(3) = (Kt*x(1) - Bsm*(x(3) - x(5))) / Jm;
    % Gear 1 dynamics
    xdot(4) = x(5);
    xdot(5) = (Bsm*(x(3)-x(5)))/Jg1;
    % Forearm Dynamics
    xdot(6) = x(7);
    xdot(7) = (Tf/Jf) - (Bsf/Jf)*x(7) - ((mf*lf)/(2*Jf))*g*sin(ThetaU+x(6));

end