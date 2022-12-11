package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp(name="Drive Test")
public class DriveTest extends LinearOpMode {
    private GamepadEx driverController;

    private DriveSubsystem driveSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        this.driverController = new GamepadEx(gamepad1);

        this.driveSubsystem = new DriveSubsystem(
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

            double driveForwardValue = this.driverController.getLeftY() * 2.0;
            double driveRightValue = this.driverController.getLeftX() * 2.0;
            double driveRotationValue = this.driverController.getRightX() * 4.0;

            this.driveSubsystem.driveTeleOp(
                    driveForwardValue,
                    driveRightValue,
                    driveRotationValue,
                    true
            );

            this.driveSubsystem.periodic();

            telemetry.update();
        }

        driveSubsystem.stop();
    }
}
