function theta_ref = maneuver_slew(theta, x, x_ref)
    direction_e = x_ref - x;
    Tb_e_psi = [-cos(theta(3)), -sin(theta(3)), 0;
                sin(theta(3)), -cos(theta(3)), 0;
                0,          0,          1];
    Tb_e_theta = [-cos(theta(2)), 0, -sin(theta(2));
                0,          1,      0;
                -sin(theta(2)), 0, -cos(theta(2))];
    Tb_e_phi = [1,      0,          0;
                0, -cos(theta(1)), -sin(theta(1));
                0, sin(theta(1)), -cos(theta(1))];
    Tb_e = Tb_e_psi * Tb_e_theta * Tb_e_phi;

    direction_b = Tb_e * direction_e;
    
    theta_ref = [0;0;0];
    theta_ref(3) = atan(direction_b(2)/direction_b(1));
    theta_ref(2) = pi/2 + atan(direction_b(3)/norm([direction_b(1), direction_b(2)]));

end
