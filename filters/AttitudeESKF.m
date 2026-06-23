classdef AttitudeESKF < handle

    properties
        SampleTime = 0.01

        QuaternionState = [1 0 0 0]
        GyroBiasState = zeros(3, 1)

        P = diag([ ...
            1e-3, 1e-3, 1e-3, ...
            1e-2, 1e-2, 1e-2])

        Q = diag([ ...
            1e-7, 1e-7, 1e-7, ...
            3.5e-8, 3.5e-8, 3.5e-8])

        AccelMeasurementVariance = 0.011^2 * ones(3, 1)
        MagMeasurementVariance = 0.009^2 * ones(3, 1)

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

        function obj = AttitudeESKF(sampleTime)

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

            obj.GravityReference = obj.GravityReference(:) / norm(obj.GravityReference);
            obj.MagneticReference = obj.MagneticReference(:) / norm(obj.MagneticReference);

        end

        function Update(obj, accelMeas, gyroMeas, magMeas)

            accelMeas = accelMeas(:);
            gyroMeas = gyroMeas(:);
            magMeas = magMeas(:);

            obj.Predict(gyroMeas);

            accelNorm = norm(accelMeas);
            magNorm = norm(magMeas);

            useAccel = accelNorm > obj.AccelNormGate(1) && accelNorm < obj.AccelNormGate(2);
            useMag = magNorm > obj.MagNormGate(1) && magNorm < obj.MagNormGate(2);

            if useAccel || useMag
                obj.Correct(accelMeas, magMeas, useAccel, useMag);
            end

        end

        function Predict(obj, gyroMeas)

            dt = obj.SampleTime;

            correctedGyro = gyroMeas - obj.GyroBiasState;

            obj.QuaternionState = IntegrateQuaternion( ...
                obj.QuaternionState, ...
                correctedGyro, ...
                dt);

            F = eye(6);
            F(1:3, 1:3) = eye(3) - obj.Skew(correctedGyro) * dt;
            F(1:3, 4:6) = -eye(3) * dt;

            obj.P = F * obj.P * F.' + obj.Q;

            obj.NormalizeState();

        end

        function Correct(obj, accelMeas, magMeas, useAccel, useMag)

            z = [];
            R = [];

            if useAccel
                accelMeas = accelMeas / norm(accelMeas);
                z = [z; accelMeas];
                R = blkdiag(R, diag(obj.AccelMeasurementVariance));
            end

            if useMag
                magMeas = magMeas / norm(magMeas);
                z = [z; magMeas];
                R = blkdiag(R, diag(obj.MagMeasurementVariance));
            end

            h = obj.MeasurementModel(obj.QuaternionState, useAccel, useMag);
            H = obj.NumericalErrorJacobian(obj.QuaternionState, useAccel, useMag);

            innovation = z - h;

            S = H * obj.P * H.' + R;
            K = obj.P * H.' / S;

            deltaX = K * innovation;

            deltaTheta = deltaX(1:3);
            deltaGyroBias = deltaX(4:6);

            obj.QuaternionState = obj.ApplyErrorToQuaternion( ...
                obj.QuaternionState, ...
                deltaTheta);

            obj.GyroBiasState = obj.GyroBiasState + deltaGyroBias;

            I = eye(size(obj.P));
            obj.P = (I - K * H) * obj.P * (I - K * H).' + K * R * K.';

            resetJacobian = eye(6);
            resetJacobian(1:3, 1:3) = eye(3) - 0.5 * obj.Skew(deltaTheta);

            obj.P = resetJacobian * obj.P * resetJacobian.';

            obj.NormalizeState();

        end

    end

    methods (Access = private)

        function h = MeasurementModel(obj, q, useAccel, useMag)

            Rnb = QuatToRotm(q);

            h = [];

            if useAccel
                accelPred = Rnb.' * obj.GravityReference;
                accelPred = accelPred / norm(accelPred);
                h = [h; accelPred];
            end

            if useMag
                magPred = Rnb.' * obj.MagneticReference;
                magPred = magPred / norm(magPred);
                h = [h; magPred];
            end

        end

        function H = NumericalErrorJacobian(obj, q, useAccel, useMag)

            h = obj.MeasurementModel(q, useAccel, useMag);

            m = numel(h);
            H = zeros(m, 6);

            dx = 1e-6;

            for i = 1:3

                deltaPlus = zeros(3, 1);
                deltaMinus = zeros(3, 1);

                deltaPlus(i) = dx;
                deltaMinus(i) = -dx;

                qPlus = obj.ApplyErrorToQuaternion(q, deltaPlus);
                qMinus = obj.ApplyErrorToQuaternion(q, deltaMinus);

                hPlus = obj.MeasurementModel(qPlus, useAccel, useMag);
                hMinus = obj.MeasurementModel(qMinus, useAccel, useMag);

                H(:, i) = (hPlus - hMinus) / (2 * dx);

            end

        end

        function qCorrected = ApplyErrorToQuaternion(obj, q, deltaTheta)

            q = q(:).';

            deltaQuat = obj.SmallAngleQuaternion(deltaTheta);

            qCorrected = QuatMultiply(q, deltaQuat);
            qCorrected = QuatNormalize(qCorrected);

        end

        function deltaQuat = SmallAngleQuaternion(~, deltaTheta)

            deltaTheta = deltaTheta(:);

            angle = norm(deltaTheta);

            if angle < 1e-12
                deltaQuat = [1, 0.5 * deltaTheta.'];
            else
                axis = deltaTheta / angle;

                deltaQuat = [ ...
                    cos(0.5 * angle), ...
                    axis.' * sin(0.5 * angle)];
            end

            deltaQuat = QuatNormalize(deltaQuat);

        end

        function S = Skew(~, v)

            v = v(:);

            S = [ ...
                0, -v(3), v(2); ...
                v(3), 0, -v(1); ...
                -v(2), v(1), 0];

        end

        function NormalizeState(obj)

            obj.QuaternionState = QuatNormalize(obj.QuaternionState);
            obj.GyroBiasState = obj.GyroBiasState(:);
            obj.P = 0.5 * (obj.P + obj.P.');

        end
    end
end
