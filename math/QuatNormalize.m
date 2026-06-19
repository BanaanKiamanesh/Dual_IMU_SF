function q = QuatNormalize(q)

    q = q(:).';

    qNorm = norm(q);

    if qNorm < eps
        q = [1 0 0 0];
    else
        q = q / qNorm;
    end

end
