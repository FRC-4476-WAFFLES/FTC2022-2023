package lib.autoNavigation.spline;


import com.arcrobotics.ftclib.spline.PoseWithCurvature;

public class PoseWithCurvatureAndRotation extends PoseWithCurvature {
    public double rotationRadiansPerMeter;

    public PoseWithCurvatureAndRotation(PoseWithCurvature pose, double rotationRadiansPerMeter) {
        super(pose.poseMeters, pose.curvatureRadPerMeter);
        this.rotationRadiansPerMeter = rotationRadiansPerMeter;
    }

    public PoseWithCurvatureAndRotation(PoseWithCurvature pose) {
        super(pose.poseMeters, pose.curvatureRadPerMeter);
    }

    public PoseWithCurvatureAndRotation() {
        super();
    }
}
