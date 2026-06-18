classdef IMU < handle

    properties
        SampleTime = 0.01

        AccelWhiteNoiseStd = 0.02 * ones(3, 1)
        GyroWhiteNoiseStd = 0.005 * ones(3, 1)

        AccelBiasInstabilityStd = 1e-4 * ones(3, 1)
        GyroBiasInstabilityStd = 1e-5 * ones(3, 1)

        AccelBias = zeros(3, 1)
        GyroBias = zeros(3, 1)

        AccelScaleFactor = ones(3, 1)
        GyroScaleFactor = ones(3, 1)

        AccelRange = 16 * 9.81
        GyroRange = deg2rad(300)

        AccelBits = 16
        GyroBits = 16

        EnableBias = true
        EnableWhiteNoise = true
        EnableBiasInstability = true
        EnableScaleFactor = true
        EnableSaturation = true
        EnableQuantization = true
    end

    methods

        function obj = IMU(sampleTime)

            if nargin > 0
                obj.SampleTime = sampleTime;
            end

        end

        function [accelMeas, gyroMeas] = Measure(obj, specificForceTrue, angularRateTrue)

            specificForceTrue = specificForceTrue(:);
            angularRateTrue = angularRateTrue(:);

            obj.UpdateBiasInstability();

            accelMeas = specificForceTrue;
            gyroMeas = angularRateTrue;

            if obj.EnableScaleFactor
                accelMeas = obj.AccelScaleFactor .* accelMeas;
                gyroMeas = obj.GyroScaleFactor .* gyroMeas;
            end

            if obj.EnableBias
                accelMeas = accelMeas + obj.AccelBias;
                gyroMeas = gyroMeas + obj.GyroBias;
            end

            if obj.EnableWhiteNoise
                accelMeas = accelMeas + obj.AccelWhiteNoiseStd .* randn(3, 1);
                gyroMeas = gyroMeas + obj.GyroWhiteNoiseStd .* randn(3, 1);
            end

            if obj.EnableSaturation
                accelMeas = obj.Saturate(accelMeas, obj.AccelRange);
                gyroMeas = obj.Saturate(gyroMeas, obj.GyroRange);
            end

            if obj.EnableQuantization
                accelMeas = obj.Quantize(accelMeas, obj.AccelRange, obj.AccelBits);
                gyroMeas = obj.Quantize(gyroMeas, obj.GyroRange, obj.GyroBits);
            end

        end

        function ResetBias(obj, accelBias, gyroBias)

            obj.AccelBias = accelBias(:);
            obj.GyroBias = gyroBias(:);

        end

        function DisableErrors(obj)

            obj.EnableBias = false;
            obj.EnableWhiteNoise = false;
            obj.EnableBiasInstability = false;
            obj.EnableScaleFactor = false;
            obj.EnableSaturation = false;
            obj.EnableQuantization = false;

        end

    end

    methods (Access = private)

        function UpdateBiasInstability(obj)

            if obj.EnableBiasInstability

                obj.AccelBias = obj.AccelBias + ...
                    obj.AccelBiasInstabilityStd .* sqrt(obj.SampleTime) .* randn(3, 1);

                obj.GyroBias = obj.GyroBias + ...
                    obj.GyroBiasInstabilityStd .* sqrt(obj.SampleTime) .* randn(3, 1);

            end

        end

        function saturatedValue = Saturate(~, value, sensorRange)

            saturatedValue = min(max(value, -sensorRange), sensorRange);

        end

        function quantizedValue = Quantize(~, value, sensorRange, bits)

            quantizationStep = 2 * sensorRange / 2^bits;

            quantizedValue = round(value / quantizationStep) * quantizationStep;

        end

    end

end