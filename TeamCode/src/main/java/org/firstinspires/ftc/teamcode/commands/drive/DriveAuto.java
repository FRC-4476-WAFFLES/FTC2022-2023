package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.ArrayList;
import java.util.List;

import lib.autoNavigation.command.MecanumControllerCommand;
import lib.autoNavigation.trajectory.OmniTrajectory;
import lib.autoNavigation.trajectory.OmniTrajectoryConfig;
import lib.autoNavigation.trajectory.OmniTrajectoryGenerator;
import lib.autoNavigation.trajectory.OmniWaypoint;
import lib.autoNavigation.trajectory.constraint.OmniMecanumDriveKinematicsConstraint;

public class DriveAuto extends MecanumControllerCommand {
    static PIDController xController = new PIDController(4.0, 0.0, 0.0);
    static PIDController yController = new PIDController(4.0, 0.0, 0.0);
    static ProfiledPIDController thetaController = new ProfiledPIDController(4.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(
                    Constants.DriveConstants.MAX_ROT_SPEED_RAD_PER_SEC,
                    Constants.DriveConstants.MAX_ROT_ACCEL_RAD_PER_SEC_SQ
            )
    );

    public DriveAuto(OmniTrajectory trajectory) {
        super(
                trajectory,
                () -> DriveSubsystem.getInstance().getOdometryLocation(),
                xController,
                yController,
                thetaController,
                (chassisSpeeds -> DriveSubsystem.getInstance().setChassisSpeeds(chassisSpeeds)),
                DriveSubsystem.getInstance()
        );
    }

    public DriveAuto(Path path) {
        this(path.trajectory);
    }

    public static class Path {
        private final List<OmniWaypoint> waypoints = new ArrayList<>();
        OmniTrajectory trajectory;

        public Path(OmniWaypoint start) {
            waypoints.add(start);
        }

        public Path(Pose2d startPose, Rotation2d pathHeading) {
            this(new OmniWaypoint(startPose, pathHeading));
        }

        public Path(double x, double y, double heading, double pathHeading) {
            this(new Pose2d(x, y, Rotation2d.fromDegrees(heading)), Rotation2d.fromDegrees(pathHeading));
        }

        public Path waypoint(OmniWaypoint waypoint) {
            waypoints.add(waypoint);
            return this;
        }

        public Path finish(OmniWaypoint end, double maxSpeed) {
            waypoints.add(end);
            OmniTrajectoryConfig config = new OmniTrajectoryConfig(
                    maxSpeed,
                    Constants.DriveConstants.MAX_ACCEL_M_PER_S_SQ,
                    Constants.DriveConstants.MAX_ROT_SPEED_RAD_PER_SEC,
                    Constants.DriveConstants.MAX_ROT_ACCEL_RAD_PER_SEC_SQ
            );

            config.addConstraint(new OmniMecanumDriveKinematicsConstraint(
                    DriveSubsystem.getInstance().kinematics,
                    maxSpeed
            ));

            this.trajectory = OmniTrajectoryGenerator.generateTrajectory(
                    waypoints,
                    config
            );

            return this;
        }

        public Path finish(Pose2d endPose, Rotation2d pathHeading, double maxSpeed) {
            return this.finish(new OmniWaypoint(endPose, pathHeading), maxSpeed);
        }

        public Path finish(double x, double y, double heading, double pathHeading, double maxSpeed) {
            return this.finish(new Pose2d(x, y, Rotation2d.fromDegrees(heading)), Rotation2d.fromDegrees(pathHeading), maxSpeed);
        }
    }
}
