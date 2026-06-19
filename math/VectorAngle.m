function angle = VectorAngle(vectorA, vectorB)

    vectorA = vectorA(:);
    vectorB = vectorB(:);

    vectorA = vectorA / norm(vectorA);
    vectorB = vectorB / norm(vectorB);

    cosAngle = dot(vectorA, vectorB);
    cosAngle = min(1, max(-1, cosAngle));

    angle = acos(cosAngle);

end
