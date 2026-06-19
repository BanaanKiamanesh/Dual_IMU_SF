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

end
