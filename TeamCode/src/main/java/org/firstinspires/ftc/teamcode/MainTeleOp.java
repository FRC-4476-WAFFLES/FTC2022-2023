package org.firstinspires.ftc.teamcode;

import android.graphics.Camera;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends LinearOpMode {
    private MotorEx frontLeftMotor;
    private MotorEx frontRightMotor;
    private MotorEx backLeftMotor;
    private MotorEx backRightMotor;

    private MotorEx leftElevatorMotor;
    private MotorEx rightElevatorMotor;
    private MotorEx armMotor;

    private MotorEx intakeMotor;

    private GyroEx gyro;

    private GamepadEx driverController;
    private GamepadEx operatorController;

    private MecanumDrive drive;

    private ElevatorArmSubsystem elevatorArm;
    private IntakeSubsystem intake;
    private CameraSubsystem camera;

    @Override
    public void runOpMode() {
        this.frontLeftMotor = new MotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_312);
        this.frontRightMotor = new MotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_312);
        this.backLeftMotor = new MotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_312);
        this.backRightMotor = new MotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_312);

        this.leftElevatorMotor = new MotorEx(hardwareMap, "leftElevator");
        this.rightElevatorMotor = new MotorEx(hardwareMap, "rightElevator");
        this.armMotor = new MotorEx(hardwareMap, "arm");

        this.intakeMotor = new MotorEx(hardwareMap, "intake");

        this.driverController = new GamepadEx(gamepad1);
        this.operatorController = new GamepadEx(gamepad2);

        this.elevatorArm = new ElevatorArmSubsystem(
                leftElevatorMotor,
                rightElevatorMotor,
                armMotor
        );
        this.intake = new IntakeSubsystem(
                intakeMotor
        );

        this.frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.frontLeftMotor.setInverted(true);
        this.frontRightMotor.setInverted(true);
        this.backLeftMotor.setInverted(true);
        this.backRightMotor.setInverted(true);

        this.intakeMotor.setRunMode(Motor.RunMode.RawPower);

        this.gyro.init();

        this.drive = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        this.elevatorArm.initialize();

        waitForStart();

        while (opModeIsActive()) {
            if (
                    this.driverController.getButton(GamepadKeys.Button.START)
                    && this.driverController.getButton(GamepadKeys.Button.BACK)
            ) {
                this.gyro.reset();
            }

            if (this.operatorController.getButton(GamepadKeys.Button.A)) {
                this.elevatorArm.setTargetLevel(ElevatorArmSubsystem.Levels.GROUNDED);
            } else if (this.operatorController.getButton(GamepadKeys.Button.B)) {
                this.elevatorArm.setTargetLevel(ElevatorArmSubsystem.Levels.L1);
            } else if (this.operatorController.getButton(GamepadKeys.Button.X)) {
                this.elevatorArm.setTargetLevel(ElevatorArmSubsystem.Levels.L2);
            } else if (this.operatorController.getButton(GamepadKeys.Button.Y)) {
                this.elevatorArm.setTargetLevel(ElevatorArmSubsystem.Levels.L3);
            }

            double elevatorJoystickValue = this.operatorController.getRightY();
            double armJoystickValue = this.operatorController.getLeftY();

            this.elevatorArm.moveElevatorWithAnalogStick(elevatorJoystickValue);
            this.elevatorArm.moveArmWithAnalogStick(armJoystickValue);

            this.drive.driveFieldCentric(
                    this.driverController.getLeftX(),
                    this.driverController.getLeftY(),
                    this.driverController.getRightX(),
                    this.gyro.getHeading()
            );

            this.elevatorArm.periodic();

            telemetry.update();
        }
    }

}
