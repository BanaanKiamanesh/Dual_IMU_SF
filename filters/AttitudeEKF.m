classdef AttitudeEKF < handle

    properties
        SampleTime = 0.01

        X = [1; 0; 0; 0; 0; 0; 0]

        P = diag([ ...
            1e-3, 1e-3, 1e-3, 1e-3, ...
            1e-2, 1e-2, 1e-2])

        Q = diag([ ...
            1e-7, 1e-7, 1e-7, 1e-7, ...
            1e-8, 1e-8, 1e-8])

        AccelMeasurementVariance = 0.015^2 * ones(3, 1)
        MagMeasurementVariance = 0.012^2 * ones(3, 1)

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

        function obj = AttitudeEKF(sampleTime)

            if nargin > 0
                obj.SampleTime = sampleTime;
            end

            obj.NormalizeReferences();

        end

        function q = get.Quaternion(obj)

            q = obj.X(1:4).';

        end

        function gyroBias = get.GyroBias(obj)

            gyroBias = obj.X(5:7);

        end

        function NormalizeReferences(obj)

            obj.GravityReference = obj.GravityReference / norm(obj.GravityReference);
            obj.MagneticReference = obj.MagneticReference / norm(obj.MagneticReference);

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

            xPred = obj.StateTransition(obj.X, gyroMeas);

            F = obj.NumericalStateJacobian(obj.X, gyroMeas);

            obj.X = xPred;
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

            h = obj.MeasurementModel(obj.X, useAccel, useMag);
            H = obj.NumericalMeasurementJacobian(obj.X, useAccel, useMag);

            innovation = z - h;

            S = H * obj.P * H.' + R;
            K = obj.P * H.' / S;

            obj.X = obj.X + K * innovation;

            I = eye(size(obj.P));
            obj.P = (I - K * H) * obj.P * (I - K * H).' + K * R * K.';

            obj.NormalizeState();

        end

    end

    methods (Access = private)

        function xNext = StateTransition(obj, x, gyroMeas)

            q = x(1:4).';
            gyroBias = x(5:7);

            correctedGyro = gyroMeas - gyroBias;

            qNext = IntegrateQuaternion(q, correctedGyro, obj.SampleTime);

            xNext = [qNext(:); gyroBias];

        end

        function h = MeasurementModel(obj, x, useAccel, useMag)

            q = x(1:4).';
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

        function F = NumericalStateJacobian(obj, x, gyroMeas)

            n = numel(x);
            F = zeros(n, n);
            dx = 1e-6;

            for i = 1:n

                xPlus = x;
                xMinus = x;

                xPlus(i) = xPlus(i) + dx;
                xMinus(i) = xMinus(i) - dx;

                xPlus(1:4) = QuatNormalize(xPlus(1:4).').';
                xMinus(1:4) = QuatNormalize(xMinus(1:4).').';

                fPlus = obj.StateTransition(xPlus, gyroMeas);
                fMinus = obj.StateTransition(xMinus, gyroMeas);

                F(:, i) = (fPlus - fMinus) / (2 * dx);

            end

        end

        function H = NumericalMeasurementJacobian(obj, x, useAccel, useMag)

            n = numel(x);
            h = obj.MeasurementModel(x, useAccel, useMag);
            m = numel(h);
            H = zeros(m, n);
            dx = 1e-6;

            for i = 1:n

                xPlus = x;
                xMinus = x;

                xPlus(i) = xPlus(i) + dx;
                xMinus(i) = xMinus(i) - dx;

                xPlus(1:4) = QuatNormalize(xPlus(1:4).').';
                xMinus(1:4) = QuatNormalize(xMinus(1:4).').';

                hPlus = obj.MeasurementModel(xPlus, useAccel, useMag);
                hMinus = obj.MeasurementModel(xMinus, useAccel, useMag);

                H(:, i) = (hPlus - hMinus) / (2 * dx);

            end

        end

        function NormalizeState(obj)

            obj.X(1:4) = QuatNormalize(obj.X(1:4).').';

            obj.P = 0.5 * (obj.P + obj.P.');

        end

    end

end
