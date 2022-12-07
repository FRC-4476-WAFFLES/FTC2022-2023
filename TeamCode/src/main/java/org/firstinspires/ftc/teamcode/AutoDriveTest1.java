package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

import java.util.ArrayList;
import java.util.List;

import lib.autoNavigation.trajectory.OmniTrajectory;
import lib.autoNavigation.trajectory.OmniTrajectoryConfig;
import lib.autoNavigation.trajectory.OmniTrajectoryGenerator;
import lib.autoNavigation.trajectory.OmniWaypoint;

@Autonomous(name = "Auto Drive Test 1")
public class AutoDriveTest1 extends LinearOpMode {
    private DriveSubsystem driveSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);

        OmniWaypoint waypoint1 = new OmniWaypoint(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0));
        OmniWaypoint waypoint2 = new OmniWaypoint(new Pose2d(1, 1, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180));
        OmniWaypoint waypoint3 = new OmniWaypoint(new Pose2d(-2, 0, Rotation2d.fromDegrees(90)), Rotation2d.fromDegrees(90));

        List<OmniWaypoint> waypointList = new ArrayList<>();
        waypointList.add(waypoint1);
        waypointList.add(waypoint2);
        waypointList.add(waypoint3);

        OmniTrajectoryConfig config = new OmniTrajectoryConfig(1.5, 3, 2, 3);

        OmniTrajectory trajectory = OmniTrajectoryGenerator.generateTrajectory(waypointList, config);

        waitForStart();

        driveSubsystem.setTrajectory(trajectory);

        while (opModeIsActive()) {
            driveSubsystem.driveAuto();
            driveSubsystem.periodic();

            telemetry.update();
        }

        driveSubsystem.stop();
    }
}
