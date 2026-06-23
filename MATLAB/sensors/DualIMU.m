classdef DualIMU < handle

    properties
        SampleTime = 0.01

        IMU1
        IMU2

        MagBiasCalibration1 = zeros(3, 1)
        MagBiasCalibration2 = zeros(3, 1)

        MagScaleFactorCalibration1 = ones(3, 1)
        MagScaleFactorCalibration2 = ones(3, 1)

        AccelWeights = ones(3, 2)
        GyroWeights = ones(3, 2)
        MagWeights = ones(3, 2)
    end

    methods

        function obj = DualIMU(sampleTime)

            if nargin > 0
                obj.SampleTime = sampleTime;
            end

            obj.IMU1 = IMU(obj.SampleTime);
            obj.IMU2 = IMU(obj.SampleTime);

        end

        function [accelFused, gyroFused, magFused, data] = Measure(obj, specificForceTrue, angularRateTrue, magneticFieldTrue)

            [accel1, gyro1, mag1Raw] = obj.IMU1.Measure( ...
                specificForceTrue, ...
                angularRateTrue, ...
                magneticFieldTrue);

            [accel2, gyro2, mag2Raw] = obj.IMU2.Measure( ...
                specificForceTrue, ...
                angularRateTrue, ...
                magneticFieldTrue);

            mag1Calibrated = obj.CalibrateMagnetometer( ...
                mag1Raw, ...
                obj.MagBiasCalibration1, ...
                obj.MagScaleFactorCalibration1);

            mag2Calibrated = obj.CalibrateMagnetometer( ...
                mag2Raw, ...
                obj.MagBiasCalibration2, ...
                obj.MagScaleFactorCalibration2);

            accelFused = obj.WeightedAverage(accel1, accel2, obj.AccelWeights);
            gyroFused = obj.WeightedAverage(gyro1, gyro2, obj.GyroWeights);
            magFused = obj.WeightedAverage(mag1Calibrated, mag2Calibrated, obj.MagWeights);

            magFusedRaw = obj.WeightedAverage(mag1Raw, mag2Raw, obj.MagWeights);

            data.Accel1 = accel1;
            data.Accel2 = accel2;
            data.AccelFused = accelFused;

            data.Gyro1 = gyro1;
            data.Gyro2 = gyro2;
            data.GyroFused = gyroFused;

            data.Mag1Raw = mag1Raw;
            data.Mag2Raw = mag2Raw;
            data.Mag1Calibrated = mag1Calibrated;
            data.Mag2Calibrated = mag2Calibrated;
            data.MagFusedRaw = magFusedRaw;
            data.MagFused = magFused;

            data.AccelDisagreement = norm(accel1 - accel2);
            data.GyroDisagreement = norm(gyro1 - gyro2);
            data.MagRawDisagreement = norm(mag1Raw - mag2Raw);
            data.MagCalibratedDisagreement = norm(mag1Calibrated - mag2Calibrated);

            data.GyroBias1 = obj.IMU1.GyroBias;
            data.GyroBias2 = obj.IMU2.GyroBias;
            data.GyroBiasFused = obj.WeightedAverage( ...
                obj.IMU1.GyroBias, ...
                obj.IMU2.GyroBias, ...
                obj.GyroWeights);

        end

    end

    methods (Access = private)

        function calibratedMag = CalibrateMagnetometer(~, magRaw, magBias, magScaleFactor)

            calibratedMag = (magRaw(:) - magBias(:)) ./ magScaleFactor(:);

        end

        function fusedValue = WeightedAverage(~, value1, value2, weights)

            value1 = value1(:);
            value2 = value2(:);

            weights = weights(:,:);

            if size(weights, 1) == 1
                weights = repmat(weights, 3, 1);
            end

            if size(weights, 2) ~= 2
                error('Weights must be a 3-by-2 matrix or a 1-by-2 row vector.')
            end

            weightSum = weights(:, 1) + weights(:, 2);

            fusedValue = (weights(:, 1) .* value1 + weights(:, 2) .* value2) ./ weightSum;

        end

    end

end
