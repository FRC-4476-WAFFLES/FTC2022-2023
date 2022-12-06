package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

import lib.autoNavigation.math.MathUtil;

@Disabled
@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends LinearOpMode {
    private GamepadEx driverController;
    private GamepadEx operatorController;

    private DriveSubsystem driveSubsystem;
    private ArmSubsystem armSubsystem;
    private IntakeSubsystem intake;

    @Override
    public void runOpMode() {
        this.driverController = new GamepadEx(gamepad1);
        this.operatorController = new GamepadEx(gamepad2);

        this.driveSubsystem = new DriveSubsystem(
                hardwareMap,
                telemetry
        );

        this.armSubsystem = new ArmSubsystem(
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
                this.armSubsystem.setTargetPosition(ArmSubsystem.Positions.LOWERED_FRONT);
            } else if (this.operatorController.getButton(GamepadKeys.Button.B)) {
                this.armSubsystem.setTargetPosition(ArmSubsystem.Positions.RAISED_FRONT);
            } else if (this.operatorController.getButton(GamepadKeys.Button.X)) {
                this.armSubsystem.setTargetPosition(ArmSubsystem.Positions.LOWERED_BACK);
            } else if (this.operatorController.getButton(GamepadKeys.Button.Y)) {
                this.armSubsystem.setTargetPosition(ArmSubsystem.Positions.RAISED_BACK);
            }

            this.armSubsystem.moveTargetPositionWithJoystick(
                    MathUtil.applyDeadband(operatorController.getRightY(), 0.05)
            );

            this.armSubsystem.periodic();
            this.driveSubsystem.periodic();

            telemetry.update();
        }

        driveSubsystem.stop();
        armSubsystem.stop();
    }
}
