% Rigid body system plant with the state vector:
% x = (theta[1:3], 
%       w_b[1:3],    
%       rho[1:3],
%       x_b[1:3], 
%       xdot_b[1:3], 
%       x[1:3], 
%       xdot[1:3])
% theta = [phi, theta, psi] (z y x implicit rotations)

function dxdt = plant(x, para, thrust, rhodot)
    torque_b = zeros(8,3);
    force_b = zeros(8,3);
    for i= 1:8
        torque_b(i,:) = cross((para.x_nozzle(i,:) - para.x_cm), thrust(i,:));
        force_b(i,:) = thrust(i,:);                                           %duplicated for additional force later  
    end
    torque_net_b = sum(torque_b);
    force_net_b = sum(force_b);

    Tw_b_thetadot = [1, sin(x(1)) * tan(x(2)), cos(x(1)) * tan(x(2));
        0, cos(x(1)), -sin(x(1));
        0, sin(x(1))/cos(x(2)), cos(x(1))/cos(x(2))]
    Tb_e_psi = [-cos(x(3)), -sin(x(3)), 0;
                sin(x(3)), -cos(x(3)), 0;
                0,          0,          1];
    Tb_e_theta = [-cos(x(2)), 0, -sin(x(2));
                0,          1,      0;
                -sin(x(2)), 0, -cos(x(2))];
    Tb_e_phi = [1,      0,          0;
                0, -cos(x(1)), -sin(x(1));
                0, sin(x(1)), -cos(x(1))];
    Tb_e = Tb_e_psi * Tb_e_theta * Tb_e_phi;

    dxdt = zeros(21,1);
    dxdt(4:6) = inv(para.I) * (torque_net_b.' - rhodot - cross(x(4:6), para.I*x(4:6)+x(7:9)));
    dxdt(1:3) = Tw_b_thetadot * x(4:6);
    dxdt(7:9) = rhodot;
   
    dxdt(10:12) = x(13:15);
    dxdt(13:15) = (force_net_b.'/para.m) - cross(x(4:6), x(13:15));
    dxdt(16:18) = Tb_e * x(13:15);
    dxdt(19:21) = Tb_e * (force_net_b.'/para.m);
end