package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import java.util.Hashtable;

public class ElevatorArmSubsystem extends SubsystemBase {
    private final MotorEx leftElevatorMotor;
    private final MotorEx rightElevatorMotor;
    private final MotorEx armMotor;

    private final double TICKS_PER_ROTATION = 1440;
    private final double MM_PER_TICK = (-30 * Math.PI) / (TICKS_PER_ROTATION);

    private final double elevatorMotorRunPower = 0.6;
    private final double armMotorRunPower = 0;

    public Levels targetLevel;

    public final Hashtable<Levels, Integer> elevatorLevels;
    public final Hashtable<Levels, Integer> armLevels;

    public ElevatorArmSubsystem(MotorEx leftElevatorMotor, MotorEx rightElevatorMotor, MotorEx armMotor) {
        this.leftElevatorMotor = leftElevatorMotor;
        this.rightElevatorMotor = rightElevatorMotor;
        this.armMotor = armMotor;

        this.elevatorLevels = new Hashtable<>();
        this.elevatorLevels.put(Levels.GROUNDED, 0);
        this.elevatorLevels.put(Levels.L1, 0);
        this.elevatorLevels.put(Levels.L2, 0);
        this.elevatorLevels.put(Levels.L3, 0);

        this.armLevels = new Hashtable<>();
        this.armLevels.put(Levels.GROUNDED, 0);
        this.armLevels.put(Levels.L1, 0);
        this.armLevels.put(Levels.L2, 0);
        this.armLevels.put(Levels.L3, 0);

        this.targetLevel = null;
    }

    public void initialize() {
        this.leftElevatorMotor.setRunMode(Motor.RunMode.PositionControl);
        this.rightElevatorMotor.setRunMode(Motor.RunMode.PositionControl);
        this.armMotor.setRunMode(Motor.RunMode.PositionControl);

        this.leftElevatorMotor.resetEncoder();
        this.rightElevatorMotor.resetEncoder();
        this.armMotor.resetEncoder();

        // set the tolerance
        this.leftElevatorMotor.setPositionTolerance(15);
        this.rightElevatorMotor.setPositionTolerance(15);
        this.armMotor.setPositionTolerance(15);

        this.leftElevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.rightElevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        if (this.targetLevel != null) {
            int elevatorTarget = this.elevatorLevels.get(targetLevel);
            int armTarget = this.armLevels.get(targetLevel);

            boolean leftElevatorCheck = this.leftElevatorMotor.atTargetPosition()
                    || this.leftElevatorMotor.getCurrentPosition() > elevatorTarget;
            if (!leftElevatorCheck) {
                this.leftElevatorMotor.set(this.elevatorMotorRunPower);
            } else {
                this.leftElevatorMotor.set(0);
            }

            boolean rightElevatorCheck = this.rightElevatorMotor.atTargetPosition()
                    || this.rightElevatorMotor.getCurrentPosition() > elevatorTarget;
            if (!rightElevatorCheck) {
                this.rightElevatorMotor.set(this.elevatorMotorRunPower);
            } else {
                this.leftElevatorMotor.set(0);
            }

            boolean armCheck = this.armMotor.atTargetPosition()
                    || this.armMotor.getCurrentPosition() > armTarget;
            if (!armCheck) {
                this.armMotor.set(this.armMotorRunPower);
            } else {
                this.armMotor.set(0);
            }

            return;
        }

        this.leftElevatorMotor.set(0);
        this.rightElevatorMotor.set(0);
        this.armMotor.set(0);
    }

    public void setTargetLevel(Levels targetLevel) {
        this.targetLevel = targetLevel;
    }

    public enum Levels {
        GROUNDED,
        L1,
        L2,
        L3
    }
}