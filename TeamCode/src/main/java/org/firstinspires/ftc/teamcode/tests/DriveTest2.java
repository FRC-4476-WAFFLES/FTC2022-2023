package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

import lib.autoNavigation.math.MathUtil;

@TeleOp(name="Drive Test 2")
public class DriveTest2 extends LinearOpMode {
    private GamepadEx driverController;

    private DriveSubsystem driveSubsystem;
    private ArmSubsystem armSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        this.driverController = new GamepadEx(gamepad1);

        this.driveSubsystem = new DriveSubsystem(
                hardwareMap,
                telemetry
        );

        armSubsystem = new ArmSubsystem(
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

            /*double driveForwardValue = this.driverController.getLeftY() * 2.0;
            double driveRightValue = this.driverController.getLeftX() * 2.0;
            double driveRotationValue = this.driverController.getRightX() * 4.0;

            this.driveSubsystem.driveTeleOp(
                    driveForwardValue,
                    driveRightValue,
                    driveRotationValue,
                    true
            );*/

            if (this.driverController.getButton(GamepadKeys.Button.A)) {
                this.armSubsystem.setTargetPosition(ArmSubsystem.Positions.LOWERED_FRONT);
            } else if (this.driverController.getButton(GamepadKeys.Button.B)) {
                this.armSubsystem.setTargetPosition(ArmSubsystem.Positions.RAISED_FRONT);
            } else if (this.driverController.getButton(GamepadKeys.Button.X)) {
                this.armSubsystem.setTargetPosition(ArmSubsystem.Positions.LOWERED_BACK);
            } else if (this.driverController.getButton(GamepadKeys.Button.Y)) {
                this.armSubsystem.setTargetPosition(ArmSubsystem.Positions.RAISED_BACK);
            }

            this.armSubsystem.moveTargetPositionWithJoystick(
                    MathUtil.applyDeadband(driverController.getRightY(), 0.05)
            );

            driveSubsystem.periodic();
            armSubsystem.periodic();

            telemetry.update();
        }

        driveSubsystem.stop();
        armSubsystem.stop();
    }
}
