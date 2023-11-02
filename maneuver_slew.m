function q_ref = maneuver_slew(v_ref)
    % identity quaternion if 0 vel
    if (norm(v_ref) == 0)
        q_ref = [1;0;0;0];
    else
        v_ref = v_ref / norm(v_ref);
        axis = cross([0;0;1], v_ref);
        angle = acos(dot([0;0;1], v_ref));
        q0 = cos(angle/2);
        axis = sin(angle/2) * axis;
        q_ref = [q0;axis];
    end
end
