function torque_net = torque_env(x_cp, x_cm, A_s, q, Fs)
    c = 3*10^8;
    t_s = cross([0, 0, -Fs], x_cp - x_cm); 
    t_s = t_s * A_s * (1 + q)/ c;
    torque_net = t_s;
end