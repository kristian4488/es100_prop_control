% actuator_RW(phi_RW)
% Produces the RW generated torque rhodot 
% from the control signal u_RW, and 
% RW tilt angle phi_RW 
% assuming pyramidal configuration with 
% RW placed on the diagonals of the xy plane.
% RW_i sits in the ith quadrant.

function [omega_RW, rho] = actuator_RW(A_RW, duty)
    omega_RW = duty;
    rho = A_RW * omega_RW;


end
