function angleWrapped = WrapToPi(angle)

    angleWrapped = mod(angle + pi, 2*pi) - pi;

end
