function F_thrust = actuator_thruster(u_thrust)
    % 136mN representative thrust at 30C
    A_thrust = [0,-0.136,0;
                0, 0, 0.136;
                0, 0, 0.136;
                0, 0.136, 0;
                0, 0.136, 0;
                0, 0, 0.136;
                0, 0, 0.136;
                0, -0.136, 0];
    F_thrust = diag(u_thrust) * A_thrust;
end