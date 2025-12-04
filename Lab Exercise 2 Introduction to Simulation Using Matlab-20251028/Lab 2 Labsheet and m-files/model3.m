% This function represents the dynamic equations for Van der Pol's oscillator 
%
% This is the DERIVATIVE SECTION of the simulation.
%
% The current time, state and input values are passed to the function as arguments
% and the function returns the state derivative.
function xdot = model3(x,u)

global mu					% global parameter transferred from main program

xdot(1,1) = 10*(-x(1) + x(2)) + u;
xdot(2,1) = 28*x(1) - x(2) - x(1)*x(3);
xdot(3,1) = -((8*x(3))/3) + x(1)*x(2);
