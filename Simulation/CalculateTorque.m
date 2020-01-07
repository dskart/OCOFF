function out = CalculateTorque(Kp, desired_pos,  current_pos, current_vel, delta_t, m)
    out = (m/delta_t^2) * (desired_pos - (((-Kp*delta_t^2)/m + 1)*current_pos + delta_t * current_vel));
end