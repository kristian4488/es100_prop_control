function F_thrust = actuator_thruster(u_thrust, thrust_max, A_thrust)
    F_thrust = diag(u_thrust) * thrust_max * A_thrust;
end