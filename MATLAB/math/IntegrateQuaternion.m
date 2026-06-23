function qNext = IntegrateQuaternion(q, omegaBody, dt)

    q = QuatNormalize(q);
    omegaBody = omegaBody(:).';

    omegaQuat = [0 omegaBody];

    qDot = 0.5 * QuatMultiply(q, omegaQuat);

    qNext = q + qDot * dt;
    qNext = QuatNormalize(qNext);

end
