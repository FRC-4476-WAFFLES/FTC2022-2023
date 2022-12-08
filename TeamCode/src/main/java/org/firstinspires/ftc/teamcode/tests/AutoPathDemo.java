package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import java.util.ArrayList;
import java.util.List;

import lib.autoNavigation.trajectory.OmniTrajectory;
import lib.autoNavigation.trajectory.OmniTrajectoryConfig;
import lib.autoNavigation.trajectory.OmniTrajectoryGenerator;
import lib.autoNavigation.trajectory.OmniWaypoint;

public class AutoPathDemo {

    public static void main(String[] args) {


        OmniWaypoint waypoint1 = new OmniWaypoint(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(90));
        OmniWaypoint waypoint2 = new OmniWaypoint(new Pose2d(2, 5, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180));

        List<OmniWaypoint> waypointList = new ArrayList<>();
        waypointList.add(waypoint1);
        waypointList.add(waypoint2);

        OmniTrajectoryConfig config = new OmniTrajectoryConfig(2, 1, 3, 1);

        OmniTrajectory trajectory = OmniTrajectoryGenerator.generateTrajectory(waypointList, config);

        System.out.println(trajectory);
    }
}
