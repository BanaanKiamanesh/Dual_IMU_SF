function dualImu = ConfigureDualIMU(config)

dualImu = DualIMU(config.SampleTime);

dualImu.IMU1 = ApplyIMUConfig(dualImu.IMU1, config.DualIMU.IMU1);
dualImu.IMU2 = ApplyIMUConfig(dualImu.IMU2, config.DualIMU.IMU2);

dualImu.MagBiasCalibration1 = config.DualIMU.MagBiasCalibration1;
dualImu.MagBiasCalibration2 = config.DualIMU.MagBiasCalibration2;

dualImu.MagScaleFactorCalibration1 = config.DualIMU.MagScaleFactorCalibration1;
dualImu.MagScaleFactorCalibration2 = config.DualIMU.MagScaleFactorCalibration2;

dualImu.AccelWeights = config.DualIMU.AccelWeights;
dualImu.GyroWeights = config.DualIMU.GyroWeights;
dualImu.MagWeights = config.DualIMU.MagWeights;

end

function imu = ApplyIMUConfig(imu, imuConfig)

imu.AccelWhiteNoiseStd = imuConfig.AccelWhiteNoiseStd;
imu.GyroWhiteNoiseStd = imuConfig.GyroWhiteNoiseStd;
imu.MagWhiteNoiseStd = imuConfig.MagWhiteNoiseStd;

imu.AccelBiasInstabilityStd = imuConfig.AccelBiasInstabilityStd;
imu.GyroBiasInstabilityStd = imuConfig.GyroBiasInstabilityStd;
imu.MagBiasInstabilityStd = imuConfig.MagBiasInstabilityStd;

imu.AccelBias = imuConfig.AccelBias;
imu.GyroBias = imuConfig.GyroBias;
imu.MagBias = imuConfig.MagBias;

imu.AccelScaleFactor = imuConfig.AccelScaleFactor;
imu.GyroScaleFactor = imuConfig.GyroScaleFactor;
imu.MagScaleFactor = imuConfig.MagScaleFactor;

imu.AccelRange = imuConfig.AccelRange;
imu.GyroRange = imuConfig.GyroRange;
imu.MagRange = imuConfig.MagRange;

imu.AccelBits = imuConfig.AccelBits;
imu.GyroBits = imuConfig.GyroBits;
imu.MagBits = imuConfig.MagBits;

imu.EnableBias = imuConfig.EnableBias;
imu.EnableWhiteNoise = imuConfig.EnableWhiteNoise;
imu.EnableBiasInstability = imuConfig.EnableBiasInstability;
imu.EnableScaleFactor = imuConfig.EnableScaleFactor;
imu.EnableSaturation = imuConfig.EnableSaturation;
imu.EnableQuantization = imuConfig.EnableQuantization;

end
