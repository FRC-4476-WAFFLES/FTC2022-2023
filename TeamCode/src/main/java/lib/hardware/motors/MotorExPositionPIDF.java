package lib.hardware.motors;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import lib.autoNavigation.math.MathUtil;

public class MotorExPositionPIDF extends MotorEx {
    private final PIDController positionController = new PIDController(1, 0, 0);

    private double encoderDpp = 1;

    private double arbitraryFeedforward = 0;

    private boolean targetIsSet = false;

    public MotorExPositionPIDF(@NonNull HardwareMap hMap, String id) {
        super(hMap, id);
    }

    public MotorExPositionPIDF(@NonNull HardwareMap hMap, String id, @NonNull GoBILDA gobildaType) {
        super(hMap, id, gobildaType);
    }

    public MotorExPositionPIDF(@NonNull HardwareMap hMap, String id, double cpr, double rpm) {
        super(hMap, id, cpr, rpm);
    }

    /**
     * Sets the distance per pulse of the encoder in units per tick.
     *
     * @param distancePerPulse the desired distance per pulse
     * @return an encoder an object with the specified distance per pulse
     */
    @Override
    public Encoder setDistancePerPulse(double distancePerPulse) {
        this.encoderDpp = distancePerPulse;
        return super.setDistancePerPulse(distancePerPulse);
    }

    @Override
    public void setRunMode(RunMode runmode) {
        super.setRunMode(runmode);
        this.runmode = runmode;
        positionController.reset();
        if (runmode == RunMode.PositionControl && !targetIsSet) {
            setTargetPosition(getCurrentPosition());
            targetIsSet = false;
        }
    }

    @Override
    public void setTargetPosition(int target) {
        setTargetDistance(target * encoderDpp);
    }

    @Override
    public void setTargetDistance(double distance) {
        targetIsSet = true;
        positionController.setSetPoint(distance);
    }

    @Override
    public void setPositionTolerance(double tolerance) {
        positionController.setTolerance(tolerance);
    }

    @Override
    public void set(double output) {
        if (runmode == RunMode.PositionControl) {
            double error = positionController.calculate(encoder.getPosition()) + arbitraryFeedforward;
            motorEx.setPower(output * error);
        } else {
            super.set(output);
        }
    }

    @Override
    @Deprecated
    public void setPositionCoefficient(double kp) {
        setP(kp);
    }

    public void setPID(double kp, double ki, double kd) {
        positionController.setPID(kp, ki, kd);
    }

    public void setP(double kp) {
        positionController.setP(kp);
    }

    public void setI(double ki) {
        positionController.setI(ki);
    }

    public void setD(double ki) {
        positionController.setD(ki);
    }

    public void setF(double kf) {
        arbitraryFeedforward = MathUtil.clamp(kf, -1.0, 1.0);
    }

    public void setIntegrationBounds(double integralMin, double integralMax) {
        positionController.setIntegrationBounds(integralMin, integralMax);
    }
}
