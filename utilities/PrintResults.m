function PrintResults(results)

    fprintf('Final roll error  [deg]: %.4f\n', rad2deg(results.Metrics.FinalRollError))
    fprintf('Final pitch error [deg]: %.4f\n', rad2deg(results.Metrics.FinalPitchError))
    fprintf('Final yaw error   [deg]: %.4f\n', rad2deg(results.Metrics.FinalYawError))
    fprintf('\n')

    fprintf('Roll RMSE  [deg]: %.4f\n', rad2deg(results.Metrics.RollRmse))
    fprintf('Pitch RMSE [deg]: %.4f\n', rad2deg(results.Metrics.PitchRmse))
    fprintf('Yaw RMSE   [deg]: %.4f\n', rad2deg(results.Metrics.YawRmse))
    fprintf('\n')

    fprintf('Final estimated fused gyro bias [deg/s]:\n')
    fprintf('%.4f %.4f %.4f\n', rad2deg(results.GyroBiasEst(:, end)))
    fprintf('\n')

    fprintf('Final true fused gyro bias [deg/s]:\n')
    fprintf('%.4f %.4f %.4f\n', rad2deg(results.GyroBiasTrue(:, end)))
    fprintf('\n')

    fprintf('Final IMU 1 true gyro bias [deg/s]:\n')
    fprintf('%.4f %.4f %.4f\n', rad2deg(results.GyroBiasTrue1(:, end)))
    fprintf('\n')

    fprintf('Final IMU 2 true gyro bias [deg/s]:\n')
    fprintf('%.4f %.4f %.4f\n', rad2deg(results.GyroBiasTrue2(:, end)))
    fprintf('\n')

    fprintf('Final fused gyro bias error [deg/s]:\n')
    fprintf('%.4f %.4f %.4f\n', rad2deg(results.Metrics.FinalGyroBiasError))
    fprintf('\n')

    fprintf('Raw fused magnetometer direction RMSE [deg]: %.4f\n', ...
        rad2deg(results.Metrics.RawMagDirectionRmse))

    fprintf('Calibrated fused magnetometer direction RMSE [deg]: %.4f\n', ...
        rad2deg(results.Metrics.CalibratedMagDirectionRmse))
    fprintf('\n')

    fprintf('Dual IMU disagreement RMS:\n')
    fprintf('Accel [m/s^2]: %.4f\n', results.Metrics.AccelDisagreementRms)
    fprintf('Gyro  [rad/s]: %.4f\n', results.Metrics.GyroDisagreementRms)
    fprintf('Mag raw [-]: %.4f\n', results.Metrics.MagRawDisagreementRms)
    fprintf('Mag cal [-]: %.4f\n', results.Metrics.MagCalibratedDisagreementRms)
    fprintf('\n')

    fprintf('Maximum quaternion norm error:\n')
    fprintf('True: %.3e\n', results.Metrics.QTrueNormErrorMax)
    fprintf('EKF : %.3e\n', results.Metrics.QEstNormErrorMax)

end
