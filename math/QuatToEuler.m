function euler = QuatToEuler(q)

    q = QuatNormalize(q);

    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);

    roll = atan2( ...
        2 * (w*x + y*z), ...
        1 - 2 * (x^2 + y^2));

    sinPitch = 2 * (w*y - z*x);
    sinPitch = min(max(sinPitch, -1), 1);

    pitch = asin(sinPitch);

    yaw = atan2( ...
        2 * (w*z + x*y), ...
        1 - 2 * (y^2 + z^2));

    euler = [roll; pitch; yaw];

end
