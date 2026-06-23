function eskf = ConfigureAttitudeESKF(config)

    eskf = AttitudeESKF(config.SampleTime);

    eskf.QuaternionState = config.ESKF.InitialQuaternion;
    eskf.GyroBiasState = config.ESKF.InitialGyroBias;

    eskf.P = config.ESKF.InitialErrorCovariance;
    eskf.Q = config.ESKF.ProcessNoise;

    eskf.AccelMeasurementVariance = config.ESKF.AccelMeasurementVariance;
    eskf.MagMeasurementVariance = config.ESKF.MagMeasurementVariance;

    eskf.GravityReference = [0; 0; 1];
    eskf.MagneticReference = config.MagneticFieldNav;

    eskf.AccelNormGate = config.ESKF.AccelNormGate;
    eskf.MagNormGate = config.ESKF.MagNormGate;

    eskf.NormalizeReferences();

end