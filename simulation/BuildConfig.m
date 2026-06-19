function config = BuildConfig()

    config.SensorMode = 'dual';
    config.FilterMode = 'ekf';

    config.RandomSeed = 2;

    config.SampleTime = 0.01;
    config.EndTime = 30;
    config.Time = 0:config.SampleTime:config.EndTime;
    config.NumberOfSteps = numel(config.Time);

    config.GravityNav = [0; 0; -9.81];

    config.MagneticFieldNav = [0.45; 0.05; 0.89];
    config.MagneticFieldNav = config.MagneticFieldNav / norm(config.MagneticFieldNav);

    imu1.AccelWhiteNoiseStd = 4.7e-2 * ones(3, 1);
    imu1.GyroWhiteNoiseStd = 3.33e-2 * ones(3, 1);
    imu1.MagWhiteNoiseStd = 0.010 * ones(3, 1);

    imu1.AccelBiasInstabilityStd = 7.36e-4 * ones(3, 1);
    imu1.GyroBiasInstabilityStd = 1.8e-3 * ones(3, 1);
    imu1.MagBiasInstabilityStd = zeros(3, 1);

    imu1.AccelBias = [0.03; -0.02; 0.04];
    imu1.GyroBias = deg2rad([0.4; -0.3; 0.2]);
    imu1.MagBias = [0.02; -0.01; 0.015];

    imu1.AccelScaleFactor = [1.002; 0.998; 1.001];
    imu1.GyroScaleFactor = [1.001; 0.999; 1.002];
    imu1.MagScaleFactor = [1.010; 0.990; 1.005];

    imu1.AccelRange = 16 * 9.81;
    imu1.GyroRange = deg2rad(300);
    imu1.MagRange = 2.0;

    imu1.AccelBits = 16;
    imu1.GyroBits = 16;
    imu1.MagBits = 16;

    imu1.EnableBias = true;
    imu1.EnableWhiteNoise = true;
    imu1.EnableBiasInstability = true;
    imu1.EnableScaleFactor = true;
    imu1.EnableSaturation = true;
    imu1.EnableQuantization = true;

    imu2 = imu1;

    imu2.AccelWhiteNoiseStd = 5.2e-2 * ones(3, 1);
    imu2.GyroWhiteNoiseStd = 2.9e-2 * ones(3, 1);
    imu2.MagWhiteNoiseStd = 0.012 * ones(3, 1);

    imu2.AccelBias = [-0.025; 0.035; -0.015];
    imu2.GyroBias = deg2rad([-0.25; 0.20; -0.15]);
    imu2.MagBias = [-0.015; 0.012; -0.010];

    imu2.AccelScaleFactor = [0.999; 1.003; 0.997];
    imu2.GyroScaleFactor = [0.998; 1.002; 0.999];
    imu2.MagScaleFactor = [0.995; 1.008; 0.992];

    config.IMU = imu1;
    config.IMU.MagBiasCalibration = imu1.MagBias;
    config.IMU.MagScaleFactorCalibration = imu1.MagScaleFactor;

    config.DualIMU.IMU1 = imu1;
    config.DualIMU.IMU2 = imu2;

    config.DualIMU.MagBiasCalibration1 = imu1.MagBias;
    config.DualIMU.MagBiasCalibration2 = imu2.MagBias;

    config.DualIMU.MagScaleFactorCalibration1 = imu1.MagScaleFactor;
    config.DualIMU.MagScaleFactorCalibration2 = imu2.MagScaleFactor;

    config.DualIMU.AccelWeights = [ ...
        1 ./ imu1.AccelWhiteNoiseStd.^2, ...
        1 ./ imu2.AccelWhiteNoiseStd.^2];

    config.DualIMU.GyroWeights = [ ...
        1 ./ imu1.GyroWhiteNoiseStd.^2, ...
        1 ./ imu2.GyroWhiteNoiseStd.^2];

    config.DualIMU.MagWeights = [ ...
        1 ./ imu1.MagWhiteNoiseStd.^2, ...
        1 ./ imu2.MagWhiteNoiseStd.^2];

    config.EKF.InitialState = [1; 0; 0; 0; 0; 0; 0];

    config.EKF.InitialCovariance = diag([ ...
        1e-3, 1e-3, 1e-3, 1e-3, ...
        1e-2, 1e-2, 1e-2]);

    config.EKF.ProcessNoise = diag([ ...
        1e-7, 1e-7, 1e-7, 1e-7, ...
        3.5e-8, 3.5e-8, 3.5e-8]);

    config.EKF.AccelMeasurementVariance = 0.011^2 * ones(3, 1);
    config.EKF.MagMeasurementVariance = 0.009^2 * ones(3, 1);

    config.EKF.AccelNormGate = [7.0, 12.5];
    config.EKF.MagNormGate = [0.5, 1.5];

    config.Mahony.InitialQuaternion = [1 0 0 0];
    config.Mahony.InitialGyroBias = zeros(3, 1);

    config.Mahony.Kp = 2.0;
    config.Mahony.Ki = 0.05;

    config.Mahony.AccelNormGate = [7.0, 12.5];
    config.Mahony.MagNormGate = [0.5, 1.5];

end