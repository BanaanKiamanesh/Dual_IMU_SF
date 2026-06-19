function metrics = ComputeMetrics(results)

    eulerError = results.EulerError;

    metrics.RollRmse = sqrt(mean(eulerError(1, :).^2));
    metrics.PitchRmse = sqrt(mean(eulerError(2, :).^2));
    metrics.YawRmse = sqrt(mean(eulerError(3, :).^2));

    metrics.FinalRollError = eulerError(1, end);
    metrics.FinalPitchError = eulerError(2, end);
    metrics.FinalYawError = eulerError(3, end);

    metrics.FinalGyroBiasError = results.GyroBiasEst(:, end) - results.GyroBiasTrue(:, end);

    metrics.RawMagDirectionRmse = sqrt(mean(results.MagDirectionErrorRaw.^2));
    metrics.CalibratedMagDirectionRmse = sqrt(mean(results.MagDirectionErrorCalibrated.^2));

    metrics.AccelDisagreementRms = sqrt(mean(results.AccelDisagreement.^2));
    metrics.GyroDisagreementRms = sqrt(mean(results.GyroDisagreement.^2));
    metrics.MagRawDisagreementRms = sqrt(mean(results.MagRawDisagreement.^2));
    metrics.MagCalibratedDisagreementRms = sqrt(mean(results.MagCalibratedDisagreement.^2));

    metrics.AccelDisagreementMax = max(results.AccelDisagreement);
    metrics.GyroDisagreementMax = max(results.GyroDisagreement);
    metrics.MagRawDisagreementMax = max(results.MagRawDisagreement);
    metrics.MagCalibratedDisagreementMax = max(results.MagCalibratedDisagreement);

    metrics.QTrueNormErrorMax = max(abs(vecnorm(results.QTrue, 2, 2) - 1));
    metrics.QEstNormErrorMax = max(abs(vecnorm(results.QEst, 2, 2) - 1));

end
