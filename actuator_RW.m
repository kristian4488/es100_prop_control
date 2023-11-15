% actuator_RW(phi_RW)
% Produces the RW generated torque rhodot 
% from the control signal u_RW, and 
% RW tilt angle phi_RW 
% assuming pyramidal configuration with 
% RW placed on the diagonals of the xy plane.
% RW_i sits in the ith quadrant.

function [omega_RW, rho] = actuator_RW(A_RW, J_RW, duty)
    %map 0.1 to 0.2 duty cycle to +-1571 rad/s = 15,000RPM max
    omega_RW = 31420 * duty - 4713;
    rho = A_RW * omega_RW * J_RW;
end
