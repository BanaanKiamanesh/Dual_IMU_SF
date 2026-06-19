clear
close all
clc

projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(projectRoot))

%% Config
config = BuildConfig();
config.SensorMode = 'dual';

%% Simulation
numberOfRuns = 50;

rollRmseDeg = zeros(numberOfRuns, 1);
pitchRmseDeg = zeros(numberOfRuns, 1);
yawRmseDeg = zeros(numberOfRuns, 1);

finalRollErrorDeg = zeros(numberOfRuns, 1);
finalPitchErrorDeg = zeros(numberOfRuns, 1);
finalYawErrorDeg = zeros(numberOfRuns, 1);

finalGyroBiasErrorDegSec = zeros(3, numberOfRuns);

accelDisagreementRms = zeros(numberOfRuns, 1);
gyroDisagreementRms = zeros(numberOfRuns, 1);
magCalibratedDisagreementRms = zeros(numberOfRuns, 1);

for runIndex = 1:numberOfRuns

    config.RandomSeed = runIndex;

    results = RunAHRSSimulation(config);

    rollRmseDeg(runIndex) = rad2deg(results.Metrics.RollRmse);
    pitchRmseDeg(runIndex) = rad2deg(results.Metrics.PitchRmse);
    yawRmseDeg(runIndex) = rad2deg(results.Metrics.YawRmse);

    finalRollErrorDeg(runIndex) = rad2deg(results.EulerError(1, end));
    finalPitchErrorDeg(runIndex) = rad2deg(results.EulerError(2, end));
    finalYawErrorDeg(runIndex) = rad2deg(results.EulerError(3, end));

    finalGyroBiasErrorDegSec(:, runIndex) = rad2deg( ...
        results.GyroBiasEst(:, end) - results.GyroBiasTrue(:, end));

    accelDisagreementRms(runIndex) = results.Metrics.AccelDisagreementRms;
    gyroDisagreementRms(runIndex) = results.Metrics.GyroDisagreementRms;
    magCalibratedDisagreementRms(runIndex) = results.Metrics.MagCalibratedDisagreementRms;

end


%% Results
fprintf('Dual-IMU Monte Carlo runs: %d\n\n', numberOfRuns)

fprintf('Roll RMSE  mean/std [deg]: %.4f / %.4f\n', mean(rollRmseDeg), std(rollRmseDeg))
fprintf('Pitch RMSE mean/std [deg]: %.4f / %.4f\n', mean(pitchRmseDeg), std(pitchRmseDeg))
fprintf('Yaw RMSE   mean/std [deg]: %.4f / %.4f\n', mean(yawRmseDeg), std(yawRmseDeg))
fprintf('\n')

fprintf('Final roll error  mean/std [deg]: %.4f / %.4f\n', mean(finalRollErrorDeg), std(finalRollErrorDeg))
fprintf('Final pitch error mean/std [deg]: %.4f / %.4f\n', mean(finalPitchErrorDeg), std(finalPitchErrorDeg))
fprintf('Final yaw error   mean/std [deg]: %.4f / %.4f\n', mean(finalYawErrorDeg), std(finalYawErrorDeg))
fprintf('\n')

fprintf('Final fused gyro bias error mean [deg/s]:\n')
fprintf('%.4f %.4f %.4f\n', mean(finalGyroBiasErrorDegSec, 2))
fprintf('\n')

fprintf('Final fused gyro bias error std [deg/s]:\n')
fprintf('%.4f %.4f %.4f\n', std(finalGyroBiasErrorDegSec, 0, 2))
fprintf('\n')

fprintf('Dual-IMU disagreement RMS mean/std:\n')
fprintf('Accel [m/s^2]: %.4f / %.4f\n', mean(accelDisagreementRms), std(accelDisagreementRms))
fprintf('Gyro  [rad/s]: %.4f / %.4f\n', mean(gyroDisagreementRms), std(gyroDisagreementRms))
fprintf('Mag cal [-]: %.4f / %.4f\n', mean(magCalibratedDisagreementRms), std(magCalibratedDisagreementRms))

figure
histogram(yawRmseDeg)
grid on
xlabel('Yaw RMSE [deg]')
ylabel('Count')
title('Dual-IMU Monte Carlo Yaw RMSE')

figure
histogram(rollRmseDeg)
grid on
xlabel('Roll RMSE [deg]')
ylabel('Count')
title('Dual-IMU Monte Carlo Roll RMSE')

figure
histogram(pitchRmseDeg)
grid on
xlabel('Pitch RMSE [deg]')
ylabel('Count')
title('Dual-IMU Monte Carlo Pitch RMSE')