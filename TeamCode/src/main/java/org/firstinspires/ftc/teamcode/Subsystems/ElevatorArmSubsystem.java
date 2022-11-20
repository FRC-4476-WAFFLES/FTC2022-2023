package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Hashtable;

public class ElevatorArmSubsystem extends SubsystemBase {
    private final MotorEx leftElevatorMotor;
    private final MotorEx rightElevatorMotor;
    private final MotorEx armMotor;

    private final double TICKS_PER_ROTATION = 1440;
    private final double MM_PER_TICK = (-30 * Math.PI) / (TICKS_PER_ROTATION);

    // Max ticks per second for analog stick control
    private final double ELEVATOR_MAX_TICKS_PER_SECOND = 20;
    private final double ARM_MAX_TICKS_PER_SECOND = 20;

    private final double elevatorMotorRunPower = 0.6;
    private final double armMotorRunPower = 0;

    private Levels targetLevel;
    private int targetElevatorPosition;
    private int targetArmPosition;

    public final Hashtable<Levels, Integer> elevatorLevels;
    public final Hashtable<Levels, Integer> armLevels;

    private final ElapsedTime elapsedTime;
    private double previousTime = 0;
    private double previousLoopTime = 0;

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
        this.targetElevatorPosition = 0;
        this.targetArmPosition = 0;

        this.elapsedTime = new ElapsedTime();
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
        this.previousLoopTime = this.elapsedTime.seconds() - this.previousTime;
        this.previousTime = this.elapsedTime.seconds();

        this.leftElevatorMotor.setTargetPosition(this.targetElevatorPosition);
        this.rightElevatorMotor.setTargetPosition(this.targetElevatorPosition);
        this.armMotor.setTargetPosition(this.targetArmPosition);

        this.leftElevatorMotor.set(
                shouldLeftElevatorMotorRun() ?
                        this.elevatorMotorRunPower : 0
        );

        this.rightElevatorMotor.set(
                shouldRightElevatorMotorRun() ?
                        this.elevatorMotorRunPower : 0
        );

        this.armMotor.set(
                shouldArmMotorRun() ?
                        this.armMotorRunPower : 0
        );

    }

    public void moveElevatorTargetPosition(int amountToMove) {
        this.targetElevatorPosition += amountToMove;
    }

    public void setElevatorTargetPosition(int position) {
        this.targetElevatorPosition = position;
    }

    public void moveArmTargetPosition(int amountToMove) {
        this.targetArmPosition += amountToMove;
    }

    public void setArmTargetPosition(int position) {
        this.targetArmPosition = position;
    }

    public void moveElevatorWithAnalogStick(double analogStickValue) {
        this.moveElevatorTargetPosition(
                (int) Math.round(analogStickValue * this.previousLoopTime * this.ELEVATOR_MAX_TICKS_PER_SECOND)
        );
    }

    public void moveArmWithAnalogStick(double analogStickValue) {
        this.moveArmTargetPosition(
                (int) Math.round(analogStickValue * this.previousLoopTime * this.ARM_MAX_TICKS_PER_SECOND)
        );
    }

    public void setTargetLevel(Levels targetLevel) {
        this.targetLevel = targetLevel;
        this.setElevatorTargetPosition(this.elevatorLevels.get(this.targetLevel));
        this.setArmTargetPosition(this.armLevels.get(this.targetLevel));
    }

    private boolean shouldLeftElevatorMotorRun() {
        return !this.leftElevatorMotor.atTargetPosition()
                || this.leftElevatorMotor.getCurrentPosition() < this.targetElevatorPosition;
    }

    private boolean shouldRightElevatorMotorRun() {
        return !this.rightElevatorMotor.atTargetPosition()
                || this.rightElevatorMotor.getCurrentPosition() < this.targetElevatorPosition;
    }

    private boolean shouldArmMotorRun() {
        return !this.armMotor.atTargetPosition()
                || this.armMotor.getCurrentPosition() < this.targetArmPosition;
    }

    public void stop() {
        this.leftElevatorMotor.setTargetPosition(this.leftElevatorMotor.getCurrentPosition());
        this.rightElevatorMotor.setTargetPosition(this.rightElevatorMotor.getCurrentPosition());
        this.armMotor.setTargetPosition(this.armMotor.getCurrentPosition());

        this.leftElevatorMotor.set(0);
        this.rightElevatorMotor.set(0);
        this.armMotor.set(0);
    }

    public enum Levels {
        GROUNDED,
        L1,
        L2,
        L3
    }
}