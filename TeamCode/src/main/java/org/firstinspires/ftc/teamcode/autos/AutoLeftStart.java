package org.firstinspires.ftc.teamcode.autos;

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

@Autonomous(name = "Left Start", preselectTeleOp = "Main TeleOp")
public class AutoLeftStart extends LinearOpMode {
    private DriveSubsystem driveSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);

        List<OmniWaypoint> waypointList = new ArrayList<>();
        waypointList.add(new OmniWaypoint(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0)));
        waypointList.add(new OmniWaypoint(new Pose2d(1, 0.1, Rotation2d.fromDegrees(90)), Rotation2d.fromDegrees(180)));
        //waypointList.add(new OmniWaypoint(new Pose2d(0, 0, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(-90)));

        OmniTrajectoryConfig config = new OmniTrajectoryConfig(1.5, 3, 2, 3);

        OmniTrajectory trajectory = OmniTrajectoryGenerator.generateTrajectory(waypointList, config);

        waitForStart();

        driveSubsystem.setTrajectory(trajectory);

        while (opModeIsActive()) {
            driveSubsystem.driveAuto();
            driveSubsystem.periodic();

            telemetry.update();

            if (driveSubsystem.isPathFinished()) {
                break;
            }
        }

        driveSubsystem.stop();

        /*
        driveSubsystem.stop();
        sleep(5000);

        List<OmniWaypoint> waypointList2 = new ArrayList<>();
        waypointList2.add(new OmniWaypoint(driveSubsystem.getOdometryLocation(), Rotation2d.fromDegrees(0)));
        waypointList2.add(new OmniWaypoint(new Pose2d(1, 1, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(-90)));

        OmniTrajectory trajectory1 = OmniTrajectoryGenerator.generateTrajectory(waypointList2, config);

        driveSubsystem.setTrajectory(trajectory1);

        while (opModeIsActive()) {
            driveSubsystem.driveAuto();
            driveSubsystem.periodic();

            telemetry.update();

            if (driveSubsystem.isPathFinished()) {
                break;
            }
        }

        driveSubsystem.stop();*/
    }
}
