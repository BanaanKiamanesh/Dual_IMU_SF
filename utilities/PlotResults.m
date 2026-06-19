function PlotResults(results)

    time = results.Time;

    eulerTruePlot = unwrap(results.EulerTrue, [], 2);
    eulerEstPlot = unwrap(results.EulerEst, [], 2);

    figure
    plot(time, rad2deg(eulerTruePlot(1, :)), 'LineWidth', 1.5)
    hold on
    plot(time, rad2deg(eulerEstPlot(1, :)), '--', 'LineWidth', 1.2)
    grid on
    xlabel('Time [s]')
    ylabel('Roll [deg]')
    legend('True', 'EKF')
    title('Roll Estimate')

    figure
    plot(time, rad2deg(eulerTruePlot(2, :)), 'LineWidth', 1.5)
    hold on
    plot(time, rad2deg(eulerEstPlot(2, :)), '--', 'LineWidth', 1.2)
    grid on
    xlabel('Time [s]')
    ylabel('Pitch [deg]')
    legend('True', 'EKF')
    title('Pitch Estimate')

    figure
    plot(time, rad2deg(eulerTruePlot(3, :)), 'LineWidth', 1.5)
    hold on
    plot(time, rad2deg(eulerEstPlot(3, :)), '--', 'LineWidth', 1.2)
    grid on
    xlabel('Time [s]')
    ylabel('Yaw [deg]')
    legend('True', 'EKF')
    title('Yaw Estimate')

    figure
    plot(time, rad2deg(results.EulerError(1, :)), 'LineWidth', 1.2)
    hold on
    plot(time, rad2deg(results.EulerError(2, :)), 'LineWidth', 1.2)
    plot(time, rad2deg(results.EulerError(3, :)), 'LineWidth', 1.2)
    grid on
    xlabel('Time [s]')
    ylabel('Attitude Error [deg]')
    legend('Roll Error', 'Pitch Error', 'Yaw Error')
    title('Dual-IMU EKF AHRS Attitude Error')

    figure
    plot(time, results.AngularRateTrue(1, :), 'LineWidth', 1.5)
    hold on
    plot(time, results.AngularRateTrue(2, :), 'LineWidth', 1.5)
    plot(time, results.AngularRateTrue(3, :), 'LineWidth', 1.5)
    plot(time, results.GyroMeas(1, :), '--', 'LineWidth', 1.0)
    plot(time, results.GyroMeas(2, :), '--', 'LineWidth', 1.0)
    plot(time, results.GyroMeas(3, :), '--', 'LineWidth', 1.0)
    grid on
    xlabel('Time [s]')
    ylabel('Angular Rate [rad/s]')
    legend( ...
        '\omega_x true', '\omega_y true', '\omega_z true', ...
        '\omega_x fused', '\omega_y fused', '\omega_z fused')
    title('Fused Gyroscope Measurement')

    figure
    plot(time, results.GyroMeas1(1, :), 'LineWidth', 1.0)
    hold on
    plot(time, results.GyroMeas2(1, :), 'LineWidth', 1.0)
    plot(time, results.GyroMeas(1, :), '--', 'LineWidth', 1.3)
    grid on
    xlabel('Time [s]')
    ylabel('x Angular Rate [rad/s]')
    legend('IMU 1', 'IMU 2', 'Fused')
    title('Dual Gyro x-Axis Fusion')

    figure
    plot(time, results.SpecificForceTrue(1, :), 'LineWidth', 1.5)
    hold on
    plot(time, results.SpecificForceTrue(2, :), 'LineWidth', 1.5)
    plot(time, results.SpecificForceTrue(3, :), 'LineWidth', 1.5)
    plot(time, results.AccelMeas(1, :), '--', 'LineWidth', 1.0)
    plot(time, results.AccelMeas(2, :), '--', 'LineWidth', 1.0)
    plot(time, results.AccelMeas(3, :), '--', 'LineWidth', 1.0)
    grid on
    xlabel('Time [s]')
    ylabel('Specific Force [m/s^2]')
    legend( ...
        'f_x true', 'f_y true', 'f_z true', ...
        'f_x fused', 'f_y fused', 'f_z fused')
    title('Fused Accelerometer Measurement')

    figure
    plot(time, results.MagFieldTrue(1, :), 'LineWidth', 1.5)
    hold on
    plot(time, results.MagFieldTrue(2, :), 'LineWidth', 1.5)
    plot(time, results.MagFieldTrue(3, :), 'LineWidth', 1.5)
    plot(time, results.MagMeasCalibrated(1, :), '--', 'LineWidth', 1.0)
    plot(time, results.MagMeasCalibrated(2, :), '--', 'LineWidth', 1.0)
    plot(time, results.MagMeasCalibrated(3, :), '--', 'LineWidth', 1.0)
    grid on
    xlabel('Time [s]')
    ylabel('Magnetic Field [normalized]')
    legend( ...
        'm_x true', 'm_y true', 'm_z true', ...
        'm_x fused calibrated', 'm_y fused calibrated', 'm_z fused calibrated')
    title('Fused Calibrated Magnetometer Measurement')

    figure
    plot(time, rad2deg(results.MagDirectionErrorRaw), 'LineWidth', 1.2)
    hold on
    plot(time, rad2deg(results.MagDirectionErrorCalibrated), '--', 'LineWidth', 1.2)
    grid on
    xlabel('Time [s]')
    ylabel('Magnetic Direction Error [deg]')
    legend('Raw Fused Magnetometer', 'Calibrated Fused Magnetometer')
    title('Fused Magnetometer Direction Error')

    figure
    plot(time, results.AccelDisagreement, 'LineWidth', 1.2)
    hold on
    plot(time, results.GyroDisagreement, 'LineWidth', 1.2)
    plot(time, results.MagCalibratedDisagreement, 'LineWidth', 1.2)
    grid on
    xlabel('Time [s]')
    ylabel('Sensor-to-Sensor Disagreement')
    legend('Accel [m/s^2]', 'Gyro [rad/s]', 'Mag calibrated [-]')
    title('Dual-IMU Disagreement Metrics')

    figure
    plot(time, rad2deg(results.GyroBiasTrue(1, :)), 'LineWidth', 1.5)
    hold on
    plot(time, rad2deg(results.GyroBiasTrue(2, :)), 'LineWidth', 1.5)
    plot(time, rad2deg(results.GyroBiasTrue(3, :)), 'LineWidth', 1.5)
    plot(time, rad2deg(results.GyroBiasEst(1, :)), '--', 'LineWidth', 1.0)
    plot(time, rad2deg(results.GyroBiasEst(2, :)), '--', 'LineWidth', 1.0)
    plot(time, rad2deg(results.GyroBiasEst(3, :)), '--', 'LineWidth', 1.0)
    grid on
    xlabel('Time [s]')
    ylabel('Gyro Bias [deg/s]')
    legend( ...
        'b_x fused true', 'b_y fused true', 'b_z fused true', ...
        'b_x estimated', 'b_y estimated', 'b_z estimated')
    title('Fused Gyroscope Bias')

end
