function config = BuildConfig()

    config.RandomSeed = 2;

    config.SampleTime = 0.01;
    config.EndTime = 30;
    config.Time = 0:config.SampleTime:config.EndTime;
    config.NumberOfSteps = numel(config.Time);

    config.GravityNav = [0; 0; -9.81];

    config.MagneticFieldNav = [0.45; 0.05; 0.89];
    config.MagneticFieldNav = config.MagneticFieldNav / norm(config.MagneticFieldNav);

    config.MagBiasCalibration = [0.02; -0.01; 0.015];
    config.MagScaleFactorCalibration = [1.010; 0.990; 1.005];

    config.IMU.AccelWhiteNoiseStd = 4.7e-2 * ones(3, 1);
    config.IMU.GyroWhiteNoiseStd = 3.33e-2 * ones(3, 1);
    config.IMU.MagWhiteNoiseStd = 0.01 * ones(3, 1);

    config.IMU.AccelBiasInstabilityStd = 7.36e-4 * ones(3, 1);
    config.IMU.GyroBiasInstabilityStd = 1.8e-3 * ones(3, 1);
    config.IMU.MagBiasInstabilityStd = zeros(3, 1);

    config.IMU.AccelBias = [0.03; -0.02; 0.04];
    config.IMU.GyroBias = deg2rad([0.4; -0.3; 0.2]);
    config.IMU.MagBias = config.MagBiasCalibration;

    config.IMU.AccelScaleFactor = [1.002; 0.998; 1.001];
    config.IMU.GyroScaleFactor = [1.001; 0.999; 1.002];
    config.IMU.MagScaleFactor = config.MagScaleFactorCalibration;

    config.IMU.AccelRange = 16 * 9.81;
    config.IMU.GyroRange = deg2rad(300);
    config.IMU.MagRange = 2.0;

    config.IMU.AccelBits = 16;
    config.IMU.GyroBits = 16;
    config.IMU.MagBits = 16;

    config.IMU.EnableBias = true;
    config.IMU.EnableWhiteNoise = true;
    config.IMU.EnableBiasInstability = true;
    config.IMU.EnableScaleFactor = true;
    config.IMU.EnableSaturation = true;
    config.IMU.EnableQuantization = true;

    config.EKF.InitialState = [1; 0; 0; 0; 0; 0; 0];

    config.EKF.InitialCovariance = diag([ ...
        1e-3, 1e-3, 1e-3, 1e-3, ...
        1e-2, 1e-2, 1e-2]);

    config.EKF.ProcessNoise = diag([ ...
        1e-7, 1e-7, 1e-7, 1e-7, ...
        3.5e-8, 3.5e-8, 3.5e-8]);

    config.EKF.AccelMeasurementVariance = 0.015^2 * ones(3, 1);
    config.EKF.MagMeasurementVariance = 0.012^2 * ones(3, 1);

    config.EKF.AccelNormGate = [7.0, 12.5];
    config.EKF.MagNormGate = [0.5, 1.5];

end
