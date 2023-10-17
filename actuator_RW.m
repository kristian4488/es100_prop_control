% actuator_RW(phi_RW)
% Produces the RW generated torque rhodot 
% from the control signal u_RW, and 
% RW tilt angle phi_RW 
% assuming pyramidal configuration with 
% RW placed on the diagonals of the xy plane.
% RW_i sits in the ith quadrant.

function rhodot = actuator_RW(A_RW, u_RW)
    rhodot = A_RW * u_RW;
end
