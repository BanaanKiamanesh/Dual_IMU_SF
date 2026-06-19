function [specificForceTrue, magneticFieldTrue] = GenerateTrueSensorInputs(qTrue, config)

    RTrue = QuatToRotm(qTrue);

    specificForceTrue = RTrue.' * (-config.GravityNav);
    magneticFieldTrue = RTrue.' * config.MagneticFieldNav;

end
