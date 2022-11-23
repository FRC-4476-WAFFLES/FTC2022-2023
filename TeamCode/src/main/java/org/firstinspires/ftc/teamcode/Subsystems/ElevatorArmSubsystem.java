package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ElevatorArmSubsystem extends SubsystemBase {
    private final MotorEx leftElevatorMotor;
    private final MotorEx rightElevatorMotor;
    private final MotorEx armMotor;

    private final double TICKS_PER_ROTATION_ELEVATOR = Motor.GoBILDA.RPM_1620.getCPR();
    private final double TICKS_PER_ROTATION_ARM = Motor.GoBILDA.RPM_1620.getCPR(); // TODO: Change this to reflect what gearbox is on the arm motor

    private final double MM_PER_TICK = (-30 * Math.PI) / (TICKS_PER_ROTATION_ELEVATOR);

    // Max ticks per second for analog stick control
    private final double ELEVATOR_MAX_TICKS_PER_SECOND = 20;
    private final double ARM_MAX_TICKS_PER_SECOND = 20;

    private final double elevatorMotorRunPower = 0.6;
    private final double armMotorRunPower = 0;

    private int targetElevatorPosition;
    private int targetArmPosition;

    private final ElapsedTime elapsedTime;
    private double previousTime = 0;
    private double previousLoopTime = 0;

    public ElevatorArmSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.leftElevatorMotor = new MotorEx(hardwareMap, "leftElevator");
        this.rightElevatorMotor = new MotorEx(hardwareMap, "rightElevator");
        this.armMotor = new MotorEx(hardwareMap, "arm");

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

        this.targetElevatorPosition = 0;
        this.targetArmPosition = 0;

        this.elapsedTime = new ElapsedTime();
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
        this.setElevatorTargetPosition(targetLevel.elevatorPos);
        this.setArmTargetPosition(targetLevel.armPos);
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
        GROUNDED (0, 0),
        L1 (0, 0),
        L2 (0, 0),
        L3 (0, 0);

        public final int elevatorPos;
        public final int armPos;

        Levels(int elevatorPos, int armPos) {
            this.elevatorPos = elevatorPos;
            this.armPos = armPos;
        }
    }
}
