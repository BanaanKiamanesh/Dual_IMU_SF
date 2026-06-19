function PrintResults(results)

    fprintf('Final roll error  [deg]: %.4f\n', rad2deg(results.Metrics.FinalRollError))
    fprintf('Final pitch error [deg]: %.4f\n', rad2deg(results.Metrics.FinalPitchError))
    fprintf('Final yaw error   [deg]: %.4f\n', rad2deg(results.Metrics.FinalYawError))
    fprintf('\n')

    fprintf('Roll RMSE  [deg]: %.4f\n', rad2deg(results.Metrics.RollRmse))
    fprintf('Pitch RMSE [deg]: %.4f\n', rad2deg(results.Metrics.PitchRmse))
    fprintf('Yaw RMSE   [deg]: %.4f\n', rad2deg(results.Metrics.YawRmse))
    fprintf('\n')

    fprintf('Final estimated gyro bias [deg/s]:\n')
    fprintf('%.4f %.4f %.4f\n', rad2deg(results.GyroBiasEst(:, end)))
    fprintf('\n')

    fprintf('Final true gyro bias [deg/s]:\n')
    fprintf('%.4f %.4f %.4f\n', rad2deg(results.GyroBiasTrue(:, end)))
    fprintf('\n')

    fprintf('Final gyro bias error [deg/s]:\n')
    fprintf('%.4f %.4f %.4f\n', rad2deg(results.Metrics.FinalGyroBiasError))
    fprintf('\n')

    fprintf('Raw magnetometer direction RMSE [deg]: %.4f\n', ...
        rad2deg(results.Metrics.RawMagDirectionRmse))

    fprintf('Calibrated magnetometer direction RMSE [deg]: %.4f\n', ...
        rad2deg(results.Metrics.CalibratedMagDirectionRmse))

end
