package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.commands.drive.DriveAuto;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.ArrayList;
import java.util.List;

import lib.autoNavigation.trajectory.OmniTrajectory;
import lib.autoNavigation.trajectory.OmniTrajectoryConfig;
import lib.autoNavigation.trajectory.OmniTrajectoryGenerator;
import lib.autoNavigation.trajectory.OmniWaypoint;

public class CommandAutoTest extends CommandOpMode {
    private DriveSubsystem driveSubsystem;

    @Override
    public void initialize() {
        driveSubsystem = DriveSubsystem.getInstance();
        driveSubsystem.init(hardwareMap, telemetry);

        List<OmniWaypoint> waypointList = new ArrayList<>();
        waypointList.add(new OmniWaypoint(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0)));
        waypointList.add(new OmniWaypoint(new Pose2d(1, 0.1, Rotation2d.fromDegrees(90)), Rotation2d.fromDegrees(180)));
        //waypointList.add(new OmniWaypoint(new Pose2d(0, 0, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(-90)));

        OmniTrajectoryConfig config = new OmniTrajectoryConfig(1.5, 3, 2, 3);

        OmniTrajectory trajectory = OmniTrajectoryGenerator.generateTrajectory(waypointList, config);

        register(driveSubsystem);
        schedule(new DriveAuto(trajectory));
    }
}
