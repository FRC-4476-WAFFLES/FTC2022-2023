package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import lib.autoNavigation.math.MathUtil;

public class ArmSubsystem extends SubsystemBase {
    private final Telemetry telemetry;

    private final MotorEx armMotor;

    private final double TICKS_PER_ROTATION_ARM = Motor.GoBILDA.RPM_435.getCPR(); // TODO: Change this to reflect what gearbox is on the arm motor

    private final double ARM_MAX_TICKS_PER_SECOND = 20;

    private int targetPosition;

    private final ElapsedTime elapsedTime = new ElapsedTime();
    private double previousTime = 0;
    private double previousLoopTime = 0;

    public ArmSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        armMotor = new MotorEx(hardwareMap, "Arm", Motor.GoBILDA.RPM_435);

        armMotor.setRunMode(Motor.RunMode.PositionControl);
        armMotor.resetEncoder();
        armMotor.setInverted(true);
        armMotor.setPositionTolerance(15);
        armMotor.setPositionCoefficient(0.01);
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        targetPosition = 0;
    }

    @Override
    public void periodic() {
        this.previousLoopTime = this.elapsedTime.seconds() - this.previousTime;
        this.previousTime = this.elapsedTime.seconds();

        this.armMotor.setTargetPosition(this.targetPosition);

        this.armMotor.set(1.0);

        telemetry.addData("arm pos", armMotor.getCurrentPosition());
        telemetry.addData("target pos", targetPosition);
    }

    public void setTargetPosition(Positions position) {
        this.targetPosition = position.getEncoderPosition();
    }

    public void moveTargetPosition(int amountToMove) {
        this.targetPosition = MathUtil.clamp(this.targetPosition + amountToMove, 0, 1400); // TODO: adjust magic limit numbers to prevent thing from colliding with other thing
    }

    public void moveTargetPositionWithJoystick(double value) {
        moveTargetPosition((int) (value * previousLoopTime * 2.0)); // TODO: adjust magic number to make thing work good
    }

    public void stop() {
        armMotor.stopMotor();
    }

    public enum Positions {
        LOWERED_FRONT (0),
        RAISED_FRONT (500),
        RAISED_BACK (900),
        LOWERED_BACK (1400);

        private final int encoderPosition;

        Positions(int encoderPosition) {
            this.encoderPosition = encoderPosition;
        }

        public int getEncoderPosition() {
            return encoderPosition;
        }
    }
}
