function results = RunAHRSSimulation(config)

    rng(config.RandomSeed)

    imu = ConfigureIMU(config);
    ekf = ConfigureAttitudeEKF(config);

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

    qTrue = zeros(nSteps, 4);
    qEst = zeros(nSteps, 4);

    eulerTrue = zeros(3, nSteps);
    eulerEst = zeros(3, nSteps);
    eulerError = zeros(3, nSteps);

    gyroBiasTrue = zeros(3, nSteps);
    gyroBiasEst = zeros(3, nSteps);

    magDirectionErrorRaw = zeros(1, nSteps);
    magDirectionErrorCalibrated = zeros(1, nSteps);

    qTrue(1, :) = [1 0 0 0];
    qEst(1, :) = ekf.Quaternion;

    for k = 1:nSteps

        t = time(k);

        angularRateTrue(:, k) = GenerateTrueMotion(t);

        [specificForceTrue(:, k), magFieldTrue(:, k)] = GenerateTrueSensorInputs( ...
            qTrue(k, :), config);

        [accelMeas(:, k), gyroMeas(:, k), magMeas(:, k)] = imu.Measure( ...
            specificForceTrue(:, k), ...
            angularRateTrue(:, k), ...
            magFieldTrue(:, k));

        magMeasCalibrated(:, k) = (magMeas(:, k) - config.MagBiasCalibration) ./ ...
            config.MagScaleFactorCalibration;

        magDirectionErrorRaw(k) = VectorAngle(magFieldTrue(:, k), magMeas(:, k));
        magDirectionErrorCalibrated(k) = VectorAngle(magFieldTrue(:, k), magMeasCalibrated(:, k));

        ekf.Update(accelMeas(:, k), gyroMeas(:, k), magMeasCalibrated(:, k));

        qEst(k, :) = ekf.Quaternion;
        gyroBiasEst(:, k) = ekf.GyroBias;
        gyroBiasTrue(:, k) = imu.GyroBias;

        eulerTrue(:, k) = QuatToEuler(qTrue(k, :));
        eulerEst(:, k) = QuatToEuler(qEst(k, :));

        eulerError(:, k) = [ ...
            WrapToPi(eulerEst(1, k) - eulerTrue(1, k)); ...
            WrapToPi(eulerEst(2, k) - eulerTrue(2, k)); ...
            WrapToPi(eulerEst(3, k) - eulerTrue(3, k))];

        if k < nSteps
            qTrue(k + 1, :) = IntegrateQuaternion(qTrue(k, :), angularRateTrue(:, k), dt);
        end

    end

    results.Config = config;
    results.Time = time;

    results.AngularRateTrue = angularRateTrue;
    results.SpecificForceTrue = specificForceTrue;
    results.MagFieldTrue = magFieldTrue;

    results.GyroMeas = gyroMeas;
    results.AccelMeas = accelMeas;
    results.MagMeas = magMeas;
    results.MagMeasCalibrated = magMeasCalibrated;

    results.QTrue = qTrue;
    results.QEst = qEst;

    results.EulerTrue = eulerTrue;
    results.EulerEst = eulerEst;
    results.EulerError = eulerError;

    results.GyroBiasTrue = gyroBiasTrue;
    results.GyroBiasEst = gyroBiasEst;

    results.MagDirectionErrorRaw = magDirectionErrorRaw;
    results.MagDirectionErrorCalibrated = magDirectionErrorCalibrated;

    results.Metrics = ComputeMetrics(results);

end
