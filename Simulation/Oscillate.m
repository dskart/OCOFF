function out = Oscillate(amplitude, period, center, total_time, delta_t)

    t = 0:delta_t:total_time;
    w = period;
    phi = 0;
    A = amplitude;
    y = A.*cos(w*t+phi);
    y = y+center ;

    out = y;
end