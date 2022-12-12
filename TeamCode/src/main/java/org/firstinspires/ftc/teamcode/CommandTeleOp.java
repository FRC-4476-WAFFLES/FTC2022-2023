package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.arm.ArmJoystickControl;
import org.firstinspires.ftc.teamcode.commands.drive.DriveTeleOp;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeIdle;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp(name = "Command TeleOp")
public class CommandTeleOp extends CommandOpMode {
    private GamepadEx driverController, operatorController;
    private DriveSubsystem driveSubsystem;
    private ArmSubsystem armSubsystem;
    private IntakeSubsystem intakeSubsystem;

    @Override
    public void initialize() {
        driveSubsystem = DriveSubsystem.getInstance();
        driveSubsystem.init(hardwareMap, telemetry);

        armSubsystem = ArmSubsystem.getInstance();
        armSubsystem.init(hardwareMap, telemetry);

        intakeSubsystem = IntakeSubsystem.getInstance();
        intakeSubsystem.init(hardwareMap, telemetry);

        driverController = new GamepadEx(gamepad1);
        operatorController = new GamepadEx(gamepad2);

        driveSubsystem.setDefaultCommand(new DriveTeleOp(
                driverController::getLeftY,
                driverController::getLeftX,
                driverController::getRightX
        ));

        armSubsystem.setDefaultCommand(new ArmJoystickControl(operatorController::getLeftY));

        intakeSubsystem.setDefaultCommand(new IntakeIdle());

        driverController.getGamepadButton(GamepadKeys.Button.START)
                .and(driverController.getGamepadButton(GamepadKeys.Button.BACK))
                .whenActive(new InstantCommand(driveSubsystem::resetGyro, driveSubsystem));

        operatorController.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                () -> armSubsystem.setTargetPosition(ArmSubsystem.Positions.LOWERED_FRONT)
        );

        operatorController.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                () -> armSubsystem.setTargetPosition(ArmSubsystem.Positions.RAISED_FRONT)
        );

        operatorController.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                () -> armSubsystem.setTargetPosition(ArmSubsystem.Positions.LOWERED_BACK)
        );

        operatorController.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                () -> armSubsystem.setTargetPosition(ArmSubsystem.Positions.RAISED_BACK)
        );

        operatorController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(
                new InstantCommand(
                        () -> intakeSubsystem.operate(IntakeSubsystem.Modes.INTAKE),
                        intakeSubsystem
                ).perpetually()
        );

        operatorController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(
                new InstantCommand(
                        () -> intakeSubsystem.operate(IntakeSubsystem.Modes.OUTTAKE),
                        intakeSubsystem
                ).perpetually()
        );
    }
}
