function q_ref = maneuver_slew(v_cur, v_ref)
    v_diff = v_ref - v_cur;
    % identity quaternion if 0 vel
    if (norm(v_diff) == 0)
        q_ref = [1;0;0;0];
    else
        v_diff = v_diff / norm(v_diff);
        angle = acos(dot([0;0;1], v_diff));
        axis = cross([0;0;1], v_diff);
        axis = axis/norm(axis);
        q0 = cos(angle/2);
        axis = sin(angle/2) * axis;
        q_ref = [q0;axis];
    end
end
