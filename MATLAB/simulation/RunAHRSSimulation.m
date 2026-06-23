function results = RunAHRSSimulation(config)

    rng(config.RandomSeed)

    sensorMode = lower(config.SensorMode);
    filterMode = lower(config.FilterMode);

    if strcmp(sensorMode, 'single')
        imu = ConfigureIMU(config);
    elseif strcmp(sensorMode, 'dual')
        dualImu = ConfigureDualIMU(config);
    else
        error('config.SensorMode must be either ''single'' or ''dual''.')
    end

    if strcmp(filterMode, 'ekf')
        ahrs = ConfigureAttitudeEKF(config);
    elseif strcmp(filterMode, 'eskf')
        ahrs = ConfigureAttitudeESKF(config);
    elseif strcmp(filterMode, 'mahony')
        ahrs = ConfigureMahonyAHRS(config);
    else
        error('config.FilterMode must be ''ekf'', ''eskf'', or ''mahony''.')
    end

    time = config.Time;
    nSteps = config.NumberOfSteps;
    dt = config.SampleTime;

    angularRateTrue = zeros(3, nSteps);
    specificForceTrue = zeros(3, nSteps);
    magFieldTrue = zeros(3, nSteps);

    gyroMeas = zeros(3, nSteps);
    accelMeas = zeros(3, nSteps);
    magMeas = zeros(3, nSteps);
    magMeasCalibrated = zeros(3, nSteps);

    accelMeas1 = zeros(3, nSteps);
    accelMeas2 = zeros(3, nSteps);

    gyroMeas1 = zeros(3, nSteps);
    gyroMeas2 = zeros(3, nSteps);

    magMeas1Raw = zeros(3, nSteps);
    magMeas2Raw = zeros(3, nSteps);

    magMeas1Calibrated = zeros(3, nSteps);
    magMeas2Calibrated = zeros(3, nSteps);

    qTrue = zeros(nSteps, 4);
    qEst = zeros(nSteps, 4);

    eulerTrue = zeros(3, nSteps);
    eulerEst = zeros(3, nSteps);
    eulerError = zeros(3, nSteps);

    gyroBiasTrue = zeros(3, nSteps);
    gyroBiasEst = zeros(3, nSteps);

    gyroBiasTrue1 = zeros(3, nSteps);
    gyroBiasTrue2 = zeros(3, nSteps);

    accelDisagreement = zeros(1, nSteps);
    gyroDisagreement = zeros(1, nSteps);
    magRawDisagreement = zeros(1, nSteps);
    magCalibratedDisagreement = zeros(1, nSteps);

    magDirectionErrorRaw = zeros(1, nSteps);
    magDirectionErrorCalibrated = zeros(1, nSteps);

    qTrue(1, :) = [1 0 0 0];
    qEst(1, :) = ahrs.Quaternion;

    for k = 1:nSteps

        t = time(k);

        angularRateTrue(:, k) = GenerateTrueMotion(t);

        [specificForceTrue(:, k), magFieldTrue(:, k)] = GenerateTrueSensorInputs( ...
            qTrue(k, :), ...
            config);

        if strcmp(sensorMode, 'single')

            [accelRaw, gyroRaw, magRaw] = imu.Measure( ...
                specificForceTrue(:, k), ...
                angularRateTrue(:, k), ...
                magFieldTrue(:, k));

            magCalibrated = (magRaw - config.IMU.MagBiasCalibration) ./ ...
                config.IMU.MagScaleFactorCalibration;

            accelMeas(:, k) = accelRaw;
            gyroMeas(:, k) = gyroRaw;
            magMeas(:, k) = magRaw;
            magMeasCalibrated(:, k) = magCalibrated;

            accelMeas1(:, k) = accelRaw;
            accelMeas2(:, k) = accelRaw;

            gyroMeas1(:, k) = gyroRaw;
            gyroMeas2(:, k) = gyroRaw;

            magMeas1Raw(:, k) = magRaw;
            magMeas2Raw(:, k) = magRaw;

            magMeas1Calibrated(:, k) = magCalibrated;
            magMeas2Calibrated(:, k) = magCalibrated;

            gyroBiasTrue(:, k) = imu.GyroBias;
            gyroBiasTrue1(:, k) = imu.GyroBias;
            gyroBiasTrue2(:, k) = imu.GyroBias;

            accelDisagreement(k) = 0;
            gyroDisagreement(k) = 0;
            magRawDisagreement(k) = 0;
            magCalibratedDisagreement(k) = 0;

        else

            [accelFused, gyroFused, magFusedCalibrated, dualData] = dualImu.Measure( ...
                specificForceTrue(:, k), ...
                angularRateTrue(:, k), ...
                magFieldTrue(:, k));

            accelMeas(:, k) = accelFused;
            gyroMeas(:, k) = gyroFused;
            magMeas(:, k) = dualData.MagFusedRaw;
            magMeasCalibrated(:, k) = magFusedCalibrated;

            accelMeas1(:, k) = dualData.Accel1;
            accelMeas2(:, k) = dualData.Accel2;

            gyroMeas1(:, k) = dualData.Gyro1;
            gyroMeas2(:, k) = dualData.Gyro2;

            magMeas1Raw(:, k) = dualData.Mag1Raw;
            magMeas2Raw(:, k) = dualData.Mag2Raw;

            magMeas1Calibrated(:, k) = dualData.Mag1Calibrated;
            magMeas2Calibrated(:, k) = dualData.Mag2Calibrated;

            gyroBiasTrue(:, k) = dualData.GyroBiasFused;
            gyroBiasTrue1(:, k) = dualData.GyroBias1;
            gyroBiasTrue2(:, k) = dualData.GyroBias2;

            accelDisagreement(k) = dualData.AccelDisagreement;
            gyroDisagreement(k) = dualData.GyroDisagreement;
            magRawDisagreement(k) = dualData.MagRawDisagreement;
            magCalibratedDisagreement(k) = dualData.MagCalibratedDisagreement;

        end

        magDirectionErrorRaw(k) = VectorAngle( ...
            magFieldTrue(:, k), ...
            magMeas(:, k));

        magDirectionErrorCalibrated(k) = VectorAngle( ...
            magFieldTrue(:, k), ...
            magMeasCalibrated(:, k));

        ahrs.Update( ...
            accelMeas(:, k), ...
            gyroMeas(:, k), ...
            magMeasCalibrated(:, k));

        qEst(k, :) = ahrs.Quaternion;
        gyroBiasEst(:, k) = ahrs.GyroBias;

        eulerTrue(:, k) = QuatToEuler(qTrue(k, :));
        eulerEst(:, k) = QuatToEuler(qEst(k, :));

        eulerError(:, k) = [ ...
            WrapToPi(eulerEst(1, k) - eulerTrue(1, k)); ...
            WrapToPi(eulerEst(2, k) - eulerTrue(2, k)); ...
            WrapToPi(eulerEst(3, k) - eulerTrue(3, k))];

        if k < nSteps
            qTrue(k + 1, :) = IntegrateQuaternion( ...
                qTrue(k, :), ...
                angularRateTrue(:, k), ...
                dt);
        end

    end

    results.Config = config;

    results.SensorMode = sensorMode;
    results.FilterMode = filterMode;

    results.Time = time;

    results.AngularRateTrue = angularRateTrue;
    results.SpecificForceTrue = specificForceTrue;
    results.MagFieldTrue = magFieldTrue;

    results.GyroMeas = gyroMeas;
    results.AccelMeas = accelMeas;
    results.MagMeas = magMeas;
    results.MagMeasCalibrated = magMeasCalibrated;

    results.AccelMeas1 = accelMeas1;
    results.AccelMeas2 = accelMeas2;

    results.GyroMeas1 = gyroMeas1;
    results.GyroMeas2 = gyroMeas2;

    results.MagMeas1Raw = magMeas1Raw;
    results.MagMeas2Raw = magMeas2Raw;

    results.MagMeas1Calibrated = magMeas1Calibrated;
    results.MagMeas2Calibrated = magMeas2Calibrated;

    results.QTrue = qTrue;
    results.QEst = qEst;

    results.EulerTrue = eulerTrue;
    results.EulerEst = eulerEst;
    results.EulerError = eulerError;

    results.GyroBiasTrue = gyroBiasTrue;
    results.GyroBiasEst = gyroBiasEst;

    results.GyroBiasTrue1 = gyroBiasTrue1;
    results.GyroBiasTrue2 = gyroBiasTrue2;

    results.AccelDisagreement = accelDisagreement;
    results.GyroDisagreement = gyroDisagreement;
    results.MagRawDisagreement = magRawDisagreement;
    results.MagCalibratedDisagreement = magCalibratedDisagreement;

    results.MagDirectionErrorRaw = magDirectionErrorRaw;
    results.MagDirectionErrorCalibrated = magDirectionErrorCalibrated;

    results.Metrics = ComputeMetrics(results);

end