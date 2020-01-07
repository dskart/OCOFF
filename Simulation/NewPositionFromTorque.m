function out = NewPositionFromTorque(torque_input, current_pos, current_vel, delta_t, m)
    out = (torque_input/m)*delta_t^2 + current_vel*delta_t + current_pos;
end