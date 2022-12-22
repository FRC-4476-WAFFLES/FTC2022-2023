package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.commands.drive.DriveAuto;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class AutoPark extends CommandOpMode {
    private DriveSubsystem driveSubsystem;
    private CameraSubsystem cameraSubsystem;

    @Override
    public void initialize() {
        driveSubsystem = DriveSubsystem.getInstance();
        driveSubsystem.init(hardwareMap, telemetry);

        cameraSubsystem = CameraSubsystem.getInstance();
        cameraSubsystem.init(hardwareMap, telemetry);

        register(driveSubsystem, cameraSubsystem);
        schedule(new ConditionalCommand(
                new DriveAuto(new DriveAuto.Path(0, 0, 0, 0).finish(1, 0, 0, 180, 1.5)),
                new DriveAuto(new DriveAuto.Path(0, 0, 0, 0).finish(1, -1, 0, 180, 1.5)),
                () -> cameraSubsystem.getParkingLocation() == 1
        ));
    }
}
