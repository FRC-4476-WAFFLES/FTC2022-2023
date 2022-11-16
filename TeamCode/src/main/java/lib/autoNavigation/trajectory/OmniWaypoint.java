package lib.autoNavigation.trajectory;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class OmniWaypoint {
    private final Pose2d pose;
    private final Rotation2d pathHeading;

    public OmniWaypoint(Pose2d pose, Rotation2d pathHeading) {
        this.pose = pose;
        this.pathHeading = pathHeading;
    }

    public Pose2d getPose() {
        return pose;
    }

    public Rotation2d getPathHeading() {
        return pathHeading;
    }
}
