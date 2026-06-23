function PrintResults(results)

    fprintf('Sensor mode: %s\n', upper(results.SensorMode))
    fprintf('Filter mode: %s\n', upper(results.FilterMode))
    fprintf('\n')

    fprintf('Final roll error  [deg]: %.4f\n', rad2deg(results.Metrics.FinalRollError))
    fprintf('Final pitch error [deg]: %.4f\n', rad2deg(results.Metrics.FinalPitchError))
    fprintf('Final yaw error   [deg]: %.4f\n', rad2deg(results.Metrics.FinalYawError))
    fprintf('\n')

    fprintf('Roll RMSE  [deg]: %.4f\n', rad2deg(results.Metrics.RollRmse))
    fprintf('Pitch RMSE [deg]: %.4f\n', rad2deg(results.Metrics.PitchRmse))
    fprintf('Yaw RMSE   [deg]: %.4f\n', rad2deg(results.Metrics.YawRmse))
    fprintf('\n')

    if strcmpi(results.SensorMode, 'dual')
        fprintf('Final estimated fused gyro bias [deg/s]:\n')
    else
        fprintf('Final estimated gyro bias [deg/s]:\n')
    end

    fprintf('%.4f %.4f %.4f\n', rad2deg(results.GyroBiasEst(:, end)))
    fprintf('\n')

    if strcmpi(results.SensorMode, 'dual')
        fprintf('Final true fused gyro bias [deg/s]:\n')
    else
        fprintf('Final true gyro bias [deg/s]:\n')
    end

    fprintf('%.4f %.4f %.4f\n', rad2deg(results.GyroBiasTrue(:, end)))
    fprintf('\n')

    if strcmpi(results.SensorMode, 'dual')

        fprintf('Final IMU 1 true gyro bias [deg/s]:\n')
        fprintf('%.4f %.4f %.4f\n', rad2deg(results.GyroBiasTrue1(:, end)))
        fprintf('\n')

        fprintf('Final IMU 2 true gyro bias [deg/s]:\n')
        fprintf('%.4f %.4f %.4f\n', rad2deg(results.GyroBiasTrue2(:, end)))
        fprintf('\n')

        fprintf('Final fused gyro bias error [deg/s]:\n')

    else

        fprintf('Final gyro bias error [deg/s]:\n')

    end

    fprintf('%.4f %.4f %.4f\n', rad2deg(results.Metrics.FinalGyroBiasError))
    fprintf('\n')

    if strcmpi(results.SensorMode, 'dual')

        fprintf('Raw fused magnetometer direction RMSE [deg]: %.4f\n', ...
            rad2deg(results.Metrics.RawMagDirectionRmse))

        fprintf('Calibrated fused magnetometer direction RMSE [deg]: %.4f\n', ...
            rad2deg(results.Metrics.CalibratedMagDirectionRmse))

    else

        fprintf('Raw magnetometer direction RMSE [deg]: %.4f\n', ...
            rad2deg(results.Metrics.RawMagDirectionRmse))

        fprintf('Calibrated magnetometer direction RMSE [deg]: %.4f\n', ...
            rad2deg(results.Metrics.CalibratedMagDirectionRmse))

    end

    fprintf('\n')

    if strcmpi(results.SensorMode, 'dual')

        fprintf('Dual IMU disagreement RMS:\n')
        fprintf('Accel [m/s^2]: %.4f\n', results.Metrics.AccelDisagreementRms)
        fprintf('Gyro  [rad/s]: %.4f\n', results.Metrics.GyroDisagreementRms)
        fprintf('Mag raw [-]: %.4f\n', results.Metrics.MagRawDisagreementRms)
        fprintf('Mag cal [-]: %.4f\n', results.Metrics.MagCalibratedDisagreementRms)
        fprintf('\n')

    end

    fprintf('Maximum quaternion norm error:\n')
    fprintf('True: %.3e\n', results.Metrics.QTrueNormErrorMax)
    fprintf('%s : %.3e\n', upper(results.FilterMode), results.Metrics.QEstNormErrorMax)

end