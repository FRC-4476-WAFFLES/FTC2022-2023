package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drive.DriveTeleOp;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp(name = "Test TeleOp 1a")
public class TeleOpTest1 extends CommandOpMode {
    private GamepadEx driverController;
    private DriveSubsystem driveSubsystem;

    @Override
    public void initialize() {
        driveSubsystem = DriveSubsystem.getInstance();
        driveSubsystem.init(hardwareMap, telemetry);

        driverController = new GamepadEx(gamepad1);

        driveSubsystem.setDefaultCommand(new DriveTeleOp(
                driverController::getLeftY,
                driverController::getLeftX,
                driverController::getRightX
        ));

        driverController.getGamepadButton(GamepadKeys.Button.A).whileActiveContinuous(new InstantCommand(driveSubsystem::stop, driveSubsystem)); // This stops the robot even when it is in hyperspeed
        driverController.getGamepadButton(GamepadKeys.Button.B).whileActiveContinuous(
                new DriveTeleOp(
                        () -> 0.0,
                        () -> 0.0,
                        () -> 0.0
                )
        );
        driverController.getGamepadButton(GamepadKeys.Button.Y).whileActiveContinuous(new InstantCommand(() -> driveSubsystem.setPowers(0.1, 0.1, 0.1, 0.1), driveSubsystem));

        driverController.getGamepadButton(GamepadKeys.Button.START)
                .and(driverController.getGamepadButton(GamepadKeys.Button.BACK))
                .whenActive(new InstantCommand(driveSubsystem::resetGyro, driveSubsystem));
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}
