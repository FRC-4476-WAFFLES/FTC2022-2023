package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends LinearOpMode {
    private GamepadEx driverController;
    private GamepadEx operatorController;

    private DriveSubsystem driveSubsystem;
    private ElevatorArmSubsystem elevatorArm;
    private IntakeSubsystem intake;
    private CameraSubsystem camera;

    @Override
    public void runOpMode() {
        this.driverController = new GamepadEx(gamepad1);
        this.operatorController = new GamepadEx(gamepad2);

        this.driveSubsystem = new DriveSubsystem(
                hardwareMap,
                telemetry
        );

        this.elevatorArm = new ElevatorArmSubsystem(
                hardwareMap,
                telemetry
        );

        this.intake = new IntakeSubsystem(
                hardwareMap,
                telemetry
        );

        waitForStart();

        while (opModeIsActive()) {
            if (
                    this.driverController.getButton(GamepadKeys.Button.START) &&
                    this.driverController.getButton(GamepadKeys.Button.BACK)
            ) {
                driveSubsystem.resetGyro();
            }

            double driveForwardValue = this.driverController.getLeftY();
            double driveRightValue = this.driverController.getLeftX();
            double driveRotationValue = this.driverController.getRightX();

            this.driveSubsystem.driveTeleOp(
                    driveForwardValue,
                    driveRightValue,
                    driveRotationValue,
                    true
            );

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

            this.driveSubsystem.periodic();
            this.elevatorArm.periodic();

            telemetry.update();
        }

        driveSubsystem.stop();
    }
}
