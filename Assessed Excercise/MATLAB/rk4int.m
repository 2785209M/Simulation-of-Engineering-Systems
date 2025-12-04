function xnew = rk4int(fhandle, xcur, dt, Va_local, thetaU)
    % Classical RK4, takes a function handle fhandle(x, Va)
    % fhandle will be the robot arm function
    k1 = fhandle(xcur, Va_local, thetaU); % Evaluate first derivative
    k2 = fhandle(xcur + 0.5*dt*k1, Va_local, thetaU); % Evaluate second derivative
    k3 = fhandle(xcur + 0.5*dt*k2, Va_local, thetaU); % Evaluate third derivative
    k4 = fhandle(xcur + dt*k3, Va_local, thetaU); % Evaluate fourth derivative
    xnew = xcur + dt*(k1 + 2*k2 + 2*k3 + k4)/6; % Averaged output 
end