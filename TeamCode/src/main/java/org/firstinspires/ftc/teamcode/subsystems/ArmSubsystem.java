package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import lib.autoNavigation.hardware.motors.MotorExPIDF;
import lib.autoNavigation.math.MathUtil;

public class ArmSubsystem extends SubsystemBase {
    private static final ArmSubsystem instance = new ArmSubsystem();

    private final ElapsedTime elapsedTime = new ElapsedTime();
    private double previousTime = 0;
    private double previousLoopTime = 0;

    private Telemetry telemetry;

    private MotorExPIDF armMotor;

    private final double TICKS_PER_DEGREE = Motor.GoBILDA.RPM_435.getCPR() * 5.0 / 360.0;
    private final int POS_HORIZONTAL = 280;
    private final double maxGravityFF = 0.04;

    private int targetPosition;

    private ArmSubsystem() {}

    public static synchronized ArmSubsystem getInstance() {
        return instance;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        armMotor = new MotorExPIDF(hardwareMap, "Arm", Motor.GoBILDA.RPM_435);

        armMotor.setRunMode(Motor.RunMode.PositionControl);
        armMotor.resetEncoder();
        armMotor.setInverted(true);
        armMotor.setPositionTolerance(15);
        armMotor.setPID(0.005, 0.0, 0.00015); // TODO: Test and set values for these coefficients
        armMotor.setFeedforward(0.0, 0.0);
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        targetPosition = 0;
    }

    @Override
    public void periodic() {
        previousLoopTime = this.elapsedTime.seconds() - this.previousTime;
        previousTime = this.elapsedTime.seconds();

        int currentPos = armMotor.getCurrentPosition();
        double degrees = (currentPos - POS_HORIZONTAL) / TICKS_PER_DEGREE;
        double radians = Math.toRadians(degrees);
        double cosineScalar = Math.cos(radians);

        armMotor.setFeedforward(maxGravityFF * cosineScalar, 0.0);

        armMotor.setTargetPosition(this.targetPosition);

        armMotor.set(1.0);

        telemetry.addData("arm pos", armMotor.getCurrentPosition());
        telemetry.addData("target pos", targetPosition);
        telemetry.addData("cosine scalar", cosineScalar);
        telemetry.addData("applied feedforward value", maxGravityFF * cosineScalar);
    }

    public void setTargetPosition(Positions position) {
        this.targetPosition = position.getEncoderPosition();
    }

    public void moveTargetPosition(int amountToMove) {
        this.targetPosition = MathUtil.clamp(this.targetPosition + amountToMove, 0, 1475); // TODO: adjust magic limit numbers to prevent thing from colliding with other thing
    }

    public void moveTargetPositionWithJoystick(double value) {
        moveTargetPosition((int) (value * previousLoopTime * 500.0)); // TODO: adjust magic number to make thing work good
    }

    public void stop() {
        armMotor.stopMotor();
    }

    public enum Positions {
        LOWERED_FRONT (100),
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