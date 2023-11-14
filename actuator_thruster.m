function F_thrust = actuator_thruster(u_thrust, thrust_max)
    % 136mN representative thrust at 30C
    A_thrust = [0,-1,0;
                0, 0, 1;
                0, 0, 1;
                0, 1, 0;
                0, 1, 0;
                0, 0, 1;
                0, 0, 1;
                0, -1, 0];
    F_thrust = diag(u_thrust) * thrust_max * A_thrust;
end