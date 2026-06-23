classdef MahonyAHRS < handle

    properties
        SampleTime = 0.01

        QuaternionState = [1 0 0 0]
        GyroBiasState = zeros(3, 1)

        Kp = 2.0
        Ki = 0.05

        GravityReference = [0; 0; 1]
        MagneticReference = [1; 0; 0]

        AccelNormGate = [7.0, 12.5]
        MagNormGate = [0.5, 1.5]
    end

    properties (Dependent)
        Quaternion
        GyroBias
    end

    methods

        function obj = MahonyAHRS(sampleTime)

            if nargin > 0
                obj.SampleTime = sampleTime;
            end

            obj.NormalizeReferences();

        end

        function q = get.Quaternion(obj)

            q = obj.QuaternionState;

        end

        function gyroBias = get.GyroBias(obj)

            gyroBias = obj.GyroBiasState;

        end

        function NormalizeReferences(obj)

            obj.GravityReference = obj.GravityReference / norm(obj.GravityReference);
            obj.MagneticReference = obj.MagneticReference / norm(obj.MagneticReference);

        end

        function Update(obj, accelMeas, gyroMeas, magMeas)

            accelMeas = accelMeas(:);
            gyroMeas = gyroMeas(:);
            magMeas = magMeas(:);

            accelNorm = norm(accelMeas);
            magNorm = norm(magMeas);

            useAccel = accelNorm > obj.AccelNormGate(1) && accelNorm < obj.AccelNormGate(2);
            useMag = magNorm > obj.MagNormGate(1) && magNorm < obj.MagNormGate(2);

            correctionError = zeros(3, 1);

            Rnb = QuatToRotm(obj.QuaternionState);

            if useAccel

                accelMeasUnit = accelMeas / accelNorm;

                accelPred = Rnb.' * obj.GravityReference;
                accelPred = accelPred / norm(accelPred);

                correctionError = correctionError + cross(accelMeasUnit, accelPred);

            end

            if useMag

                magMeasUnit = magMeas / magNorm;

                magPred = Rnb.' * obj.MagneticReference;
                magPred = magPred / norm(magPred);

                correctionError = correctionError + cross(magMeasUnit, magPred);

            end

            if useAccel || useMag
                obj.GyroBiasState = obj.GyroBiasState - obj.Ki * correctionError * obj.SampleTime;
            end

            correctedGyro = gyroMeas - obj.GyroBiasState + obj.Kp * correctionError;

            obj.QuaternionState = IntegrateQuaternion( ...
                obj.QuaternionState, ...
                correctedGyro, ...
                obj.SampleTime);

        end
    end
end