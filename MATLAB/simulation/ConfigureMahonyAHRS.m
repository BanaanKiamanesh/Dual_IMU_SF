function mahony = ConfigureMahonyAHRS(config)

    mahony = MahonyAHRS(config.SampleTime);

    mahony.QuaternionState = config.Mahony.InitialQuaternion;
    mahony.GyroBiasState = config.Mahony.InitialGyroBias;

    mahony.Kp = config.Mahony.Kp;
    mahony.Ki = config.Mahony.Ki;

    mahony.GravityReference = [0; 0; 1];
    mahony.MagneticReference = config.MagneticFieldNav;

    mahony.AccelNormGate = config.Mahony.AccelNormGate;
    mahony.MagNormGate = config.Mahony.MagNormGate;

    mahony.NormalizeReferences();

end