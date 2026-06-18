clear
close all
clc

rng(2)

dt = 0.01;
tEnd = 30;
time = 0:dt:tEnd;
nSteps = numel(time);

imu = IMU(dt);

imu.AccelWhiteNoiseStd = 4.7e-2 * ones(3, 1);
imu.GyroWhiteNoiseStd = 3.33e-2 * ones(3, 1);

imu.AccelBiasInstabilityStd = 7.36e-4 * ones(3, 1);
imu.GyroBiasInstabilityStd = 1.8e-3 * ones(3, 1);

imu.AccelBias = [0.03; -0.02; 0.04];
imu.GyroBias = deg2rad([0.4; -0.3; 0.2]);

imu.AccelRange = 16 * 9.81;
imu.GyroRange = deg2rad(300);

imu.AccelBits = 16;
imu.GyroBits = 16;

imu.AccelScaleFactor = [1.002; 0.998; 1.001];
imu.GyroScaleFactor = [1.001; 0.999; 1.002];

imu.EnableBias = true;
imu.EnableWhiteNoise = true;
imu.EnableBiasInstability = true;
imu.EnableScaleFactor = true;
imu.EnableSaturation = true;
imu.EnableQuantization = true;

specificForceTrue = zeros(3, nSteps);
angularRateTrue = zeros(3, nSteps);

accelMeas = zeros(3, nSteps);
gyroMeas = zeros(3, nSteps);

accelBiasHistory = zeros(3, nSteps);
gyroBiasHistory = zeros(3, nSteps);

for k = 1:nSteps

    t = time(k);

    angularRateTrue(:, k) = [ ...
        deg2rad(20) * sin(0.8 * t); ...
        deg2rad(15) * cos(0.5 * t); ...
        deg2rad(10) + deg2rad(5) * sin(0.3 * t)];

    specificForceTrue(:, k) = [ ...
        0.8 * sin(0.4 * t); ...
        0.5 * cos(0.6 * t); ...
        9.81 + 0.2 * sin(0.3 * t)];

    [accelMeas(:, k), gyroMeas(:, k)] = imu.Measure( ...
        specificForceTrue(:, k), ...
        angularRateTrue(:, k));

    accelBiasHistory(:, k) = imu.AccelBias;
    gyroBiasHistory(:, k) = imu.GyroBias;

end

accelError = accelMeas - specificForceTrue;
gyroError = gyroMeas - angularRateTrue;

fprintf('Accelerometer error mean [m/s^2]:\n')
fprintf('%.6f %.6f %.6f\n', mean(accelError, 2))

fprintf('Accelerometer error std [m/s^2]:\n')
fprintf('%.6f %.6f %.6f\n', std(accelError, 0, 2))

fprintf('Gyroscope error mean [rad/s]:\n')
fprintf('%.6f %.6f %.6f\n', mean(gyroError, 2))

fprintf('Gyroscope error std [rad/s]:\n')
fprintf('%.6f %.6f %.6f\n', std(gyroError, 0, 2))

figure
plot(time, angularRateTrue(1, :), 'LineWidth', 1.5)
hold on
plot(time, angularRateTrue(2, :), 'LineWidth', 1.5)
plot(time, angularRateTrue(3, :), 'LineWidth', 1.5)
plot(time, gyroMeas(1, :), '--', 'LineWidth', 1.0)
plot(time, gyroMeas(2, :), '--', 'LineWidth', 1.0)
plot(time, gyroMeas(3, :), '--', 'LineWidth', 1.0)
grid on
xlabel('Time [s]')
ylabel('Angular Rate [rad/s]')
legend( ...
    '\omega_x true', '\omega_y true', '\omega_z true', ...
    '\omega_x measured', '\omega_y measured', '\omega_z measured')
title('Gyroscope Measurement')

figure
plot(time, specificForceTrue(1, :), 'LineWidth', 1.5)
hold on
plot(time, specificForceTrue(2, :), 'LineWidth', 1.5)
plot(time, specificForceTrue(3, :), 'LineWidth', 1.5)
plot(time, accelMeas(1, :), '--', 'LineWidth', 1.0)
plot(time, accelMeas(2, :), '--', 'LineWidth', 1.0)
plot(time, accelMeas(3, :), '--', 'LineWidth', 1.0)
grid on
xlabel('Time [s]')
ylabel('Specific Force [m/s^2]')
legend( ...
    'f_x true', 'f_y true', 'f_z true', ...
    'f_x measured', 'f_y measured', 'f_z measured')
title('Accelerometer Measurement')

figure
plot(time, gyroError(1, :), 'LineWidth', 1.2)
hold on
plot(time, gyroError(2, :), 'LineWidth', 1.2)
plot(time, gyroError(3, :), 'LineWidth', 1.2)
grid on
xlabel('Time [s]')
ylabel('Gyro Error [rad/s]')
legend('e_{\omega x}', 'e_{\omega y}', 'e_{\omega z}')
title('Gyroscope Error')

figure
plot(time, accelError(1, :), 'LineWidth', 1.2)
hold on
plot(time, accelError(2, :), 'LineWidth', 1.2)
plot(time, accelError(3, :), 'LineWidth', 1.2)
grid on
xlabel('Time [s]')
ylabel('Accelerometer Error [m/s^2]')
legend('e_{f x}', 'e_{f y}', 'e_{f z}')
title('Accelerometer Error')

figure
plot(time, rad2deg(gyroBiasHistory(1, :)), 'LineWidth', 1.2)
hold on
plot(time, rad2deg(gyroBiasHistory(2, :)), 'LineWidth', 1.2)
plot(time, rad2deg(gyroBiasHistory(3, :)), 'LineWidth', 1.2)
grid on
xlabel('Time [s]')
ylabel('Gyro Bias [deg/s]')
legend('b_{\omega x}', 'b_{\omega y}', 'b_{\omega z}')
title('Gyroscope Bias History')

figure
plot(time, accelBiasHistory(1, :), 'LineWidth', 1.2)
hold on
plot(time, accelBiasHistory(2, :), 'LineWidth', 1.2)
plot(time, accelBiasHistory(3, :), 'LineWidth', 1.2)
grid on
xlabel('Time [s]')
ylabel('Accel Bias [m/s^2]')
legend('b_{f x}', 'b_{f y}', 'b_{f z}')
title('Accelerometer Bias History')