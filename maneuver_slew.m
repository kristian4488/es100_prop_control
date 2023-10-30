function q_ref = maneuver_slew(x, x_ref)
    direction_e = x_ref - x;
    direction_e = direction_e / norm(direction_e);
    axis = cross([0;0;1], direction_e);
    angle = acos(dot([0;0;1], direction_e));
    q0 = cos(angle/2);
    axis = sin(angle/2) * axis;
    q_ref = [q0;axis];
end
