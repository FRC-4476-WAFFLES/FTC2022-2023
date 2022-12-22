package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.commands.drive.DriveAuto;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class CommandAutoTest extends CommandOpMode {
    private DriveSubsystem driveSubsystem;

    @Override
    public void initialize() {
        driveSubsystem = DriveSubsystem.getInstance();
        driveSubsystem.init(hardwareMap, telemetry);

        register(driveSubsystem);
        schedule(new DriveAuto(
                new DriveAuto.Path(0, 0, 0, 0)
                .finish(1, 0.1, 90, 90, 1.5))
        );
    }
}
