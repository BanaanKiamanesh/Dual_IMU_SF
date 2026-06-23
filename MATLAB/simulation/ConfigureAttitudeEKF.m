function ekf = ConfigureAttitudeEKF(config)

    ekf = AttitudeEKF(config.SampleTime);

    ekf.X = config.EKF.InitialState;
    ekf.P = config.EKF.InitialCovariance;
    ekf.Q = config.EKF.ProcessNoise;

    ekf.AccelMeasurementVariance = config.EKF.AccelMeasurementVariance;
    ekf.MagMeasurementVariance = config.EKF.MagMeasurementVariance;

    ekf.GravityReference = [0; 0; 1];
    ekf.MagneticReference = config.MagneticFieldNav;

    ekf.AccelNormGate = config.EKF.AccelNormGate;
    ekf.MagNormGate = config.EKF.MagNormGate;

    ekf.NormalizeReferences();

end
