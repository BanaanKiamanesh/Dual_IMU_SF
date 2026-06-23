function imu = ConfigureIMU(config)

    imu = IMU(config.SampleTime);

    imu.AccelWhiteNoiseStd = config.IMU.AccelWhiteNoiseStd;
    imu.GyroWhiteNoiseStd = config.IMU.GyroWhiteNoiseStd;
    imu.MagWhiteNoiseStd = config.IMU.MagWhiteNoiseStd;

    imu.AccelBiasInstabilityStd = config.IMU.AccelBiasInstabilityStd;
    imu.GyroBiasInstabilityStd = config.IMU.GyroBiasInstabilityStd;
    imu.MagBiasInstabilityStd = config.IMU.MagBiasInstabilityStd;

    imu.AccelBias = config.IMU.AccelBias;
    imu.GyroBias = config.IMU.GyroBias;
    imu.MagBias = config.IMU.MagBias;

    imu.AccelScaleFactor = config.IMU.AccelScaleFactor;
    imu.GyroScaleFactor = config.IMU.GyroScaleFactor;
    imu.MagScaleFactor = config.IMU.MagScaleFactor;

    imu.AccelRange = config.IMU.AccelRange;
    imu.GyroRange = config.IMU.GyroRange;
    imu.MagRange = config.IMU.MagRange;

    imu.AccelBits = config.IMU.AccelBits;
    imu.GyroBits = config.IMU.GyroBits;
    imu.MagBits = config.IMU.MagBits;

    imu.EnableBias = config.IMU.EnableBias;
    imu.EnableWhiteNoise = config.IMU.EnableWhiteNoise;
    imu.EnableBiasInstability = config.IMU.EnableBiasInstability;
    imu.EnableScaleFactor = config.IMU.EnableScaleFactor;
    imu.EnableSaturation = config.IMU.EnableSaturation;
    imu.EnableQuantization = config.IMU.EnableQuantization;

end
