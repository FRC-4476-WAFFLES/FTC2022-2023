package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

import lib.autoNavigation.math.MathUtil;

//@Disabled
@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends LinearOpMode {
    private GamepadEx driverController;
    private GamepadEx operatorController;

    private DriveSubsystem driveSubsystem;
    private ArmSubsystem armSubsystem;
    private IntakeSubsystem intakeSubsystem;

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

        this.intakeSubsystem = new IntakeSubsystem(
                hardwareMap,
                telemetry
        );

        waitForStart();

        while (opModeIsActive()) {
            if (
                    driverController.getButton(GamepadKeys.Button.START) &&
                    driverController.getButton(GamepadKeys.Button.BACK)
            ) {
                driveSubsystem.resetGyro();
            }

            double driveForwardValue = driverController.getLeftY() * 1.5;
            double driveRightValue = driverController.getLeftX() * -1.5;
            double driveRotationValue = driverController.getRightX() * -3.0;

            driveSubsystem.driveTeleOp(
                    driveForwardValue,
                    driveRightValue,
                    driveRotationValue,
                    true
            );

            if (operatorController.getButton(GamepadKeys.Button.A)) {
                armSubsystem.setTargetPosition(ArmSubsystem.Positions.LOWERED_FRONT);
            } else if (operatorController.getButton(GamepadKeys.Button.B)) {
                armSubsystem.setTargetPosition(ArmSubsystem.Positions.RAISED_FRONT);
            } else if (operatorController.getButton(GamepadKeys.Button.X)) {
                armSubsystem.setTargetPosition(ArmSubsystem.Positions.LOWERED_BACK);
            } else if (operatorController.getButton(GamepadKeys.Button.Y)) {
                armSubsystem.setTargetPosition(ArmSubsystem.Positions.RAISED_BACK);
            }

            armSubsystem.moveTargetPositionWithJoystick(
                    MathUtil.applyDeadband(operatorController.getLeftY(), 0.05)
            );

            if (operatorController.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                intakeSubsystem.operate(IntakeSubsystem.Modes.INTAKE);
            } else if (operatorController.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                intakeSubsystem.operate(IntakeSubsystem.Modes.OUTTAKE);
            } else {
                intakeSubsystem.operate(IntakeSubsystem.Modes.PASSIVE);
            }

            this.armSubsystem.periodic();
            this.driveSubsystem.periodic();

            telemetry.update();
        }

        driveSubsystem.stop();
        armSubsystem.stop();
        intakeSubsystem.stop();
    }
}
