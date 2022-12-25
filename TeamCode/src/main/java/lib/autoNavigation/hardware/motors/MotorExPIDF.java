package lib.autoNavigation.hardware.motors;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorExPIDF extends MotorEx {
    private final PIDController unifiedController = new PIDController(1, 0, 0);
    private SimpleMotorFeedforward unifiedFeedforward = new SimpleMotorFeedforward(0, 0, 0);

    private double encoderDpp = 1;

    private boolean targetIsSet = false;

    public MotorExPIDF(@NonNull HardwareMap hMap, String id) {
        super(hMap, id);
    }

    public MotorExPIDF(@NonNull HardwareMap hMap, String id, @NonNull GoBILDA gobildaType) {
        super(hMap, id, gobildaType);
    }

    public MotorExPIDF(@NonNull HardwareMap hMap, String id, double cpr, double rpm) {
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
        unifiedController.reset();
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
        switch (runmode){
            case VelocityControl:
                double targetSpeed = output * ACHIEVABLE_MAX_TICKS_PER_SECOND;
                double velocityError = unifiedController.calculate(getVelocity(), targetSpeed) + unifiedFeedforward.calculate(targetSpeed);
                motorEx.setPower(velocityError / ACHIEVABLE_MAX_TICKS_PER_SECOND);
                break;
            case PositionControl:
                double positionError = unifiedController.calculate(encoder.getPosition()) + unifiedFeedforward.calculate(0);
                motorEx.setPower(output * positionError);
                break;
            case RawPower:
                motorEx.setPower(output);
                break;
        }
    }

    /**
     * @param velocity the velocity in ticks per second
     */
    @Override
    public void setVelocity(double velocity) {
        set(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND);
    }

    public void setPID(double kp, double ki, double kd) {
        unifiedController.setPID(kp, ki, kd);
    }

    public void setP(double kp) {
        unifiedController.setP(kp);
    }

    public void setI(double ki) {
        unifiedController.setI(ki);
    }

    public void setD(double kd) {
        unifiedController.setD(kd);
    }

    public void setIntegrationBounds(double integralMin, double integralMax) {
        unifiedController.setIntegrationBounds(integralMin, integralMax);
    }

    public void setFeedforward(double ks, double kv) {
        unifiedFeedforward = new SimpleMotorFeedforward(ks, kv);
    }

    /**
     * Set the proportional gain for the position controller.
     *
     * @param kp the proportional gain
     *
     * @deprecated Use {@link #setP(double)} and set control mode to PositionControl
     */
    @Override
    @Deprecated
    public void setPositionCoefficient(double kp) {
        setP(kp);
    }

    /**
     * Set the velocity pid coefficients for the motor.
     *
     * @param kp the proportional gain
     * @param ki the integral gain
     * @param kd the derivative gain
     *
     * @deprecated Use {@link #setPID(double, double, double)} and set control mode to VelocityControl
     */
    @Deprecated
    @Override
    public void setVeloCoefficients(double kp, double ki, double kd) {
        setPID(kp, ki, kd);
    }

    /**
     * Set the feedforward coefficients for the motor.
     *
     * @param ks the static gain
     * @param kv the velocity gain
     * @param ka the acceleration gain
     *
     * @deprecated Use {@link #setFeedforward(double, double)}
     */
    @Deprecated
    @Override
    public void setFeedforwardCoefficients(double ks, double kv, double ka) {
        setFeedforward(ks, kv);
    }

    /**
     * Set the feedforward coefficients for the motor.
     *
     * @param ks the static gain
     * @param kv the velocity gain
     *
     * @deprecated Use {@link #setFeedforward(double, double)}
     */
    @Deprecated
    @Override
    public void setFeedforwardCoefficients(double ks, double kv) {
        setFeedforward(ks, kv);
    }
}
