package lib.autoNavigation.trajectory;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.spline.PoseWithCurvature;
import com.arcrobotics.ftclib.spline.Spline;
import com.arcrobotics.ftclib.spline.SplineHelper;
import com.arcrobotics.ftclib.spline.SplineParameterizer;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.BiConsumer;

import lib.autoNavigation.spline.PoseWithCurvatureAndRotation;

public class OmniTrajectoryGenerator {
    private static final OmniTrajectory kDoNothingTrajectory =
            new OmniTrajectory(Collections.singletonList(new OmniTrajectory.State()));
    private static BiConsumer<String, StackTraceElement[]> errorFunc;

    /**
     * Private constructor because this is a utility class.
     */
    private OmniTrajectoryGenerator() {}

    private static void reportError(String error, StackTraceElement[] stackTrace) {
        if (errorFunc != null) {
            errorFunc.accept(error, stackTrace);
        }
    }

    /**
     * Set error reporting function. By default, DriverStation.reportError() is used.
     *
     * @param func Error reporting function, arguments are error and stackTrace.
     */
    public static void setErrorHandler(BiConsumer<String, StackTraceElement[]> func) {
        errorFunc = func;
    }

    public static OmniTrajectory generateTrajectory(List<Pose2d> poses, List<Rotation2d> pathHeadings, OmniTrajectoryConfig config) {
        List<Pose2d> newWaypoints = new ArrayList<>();
        List<Rotation2d> robotHeadings = new ArrayList<>();
        
        for (int x = 0; x < poses.size(); x++) {
            newWaypoints.add(new Pose2d(poses.get(x).getTranslation(), pathHeadings.get(x)));
            robotHeadings.add(poses.get(x).getRotation());
        }
    
        // Get the spline points
        List<PoseWithCurvatureAndRotation> points;
        try {
            points = splinePointsFromSplines(SplineHelper.getQuinticSplinesFromControlVectors(SplineHelper.getQuinticControlVectorsFromWaypoints(newWaypoints).toArray(new Spline.ControlVector[0])), robotHeadings);
        } catch (SplineParameterizer.MalformedSplineException ex) {
          reportError(ex.getMessage(), ex.getStackTrace());
          return kDoNothingTrajectory;
        }
    
        return OmniTrajectoryParameterizer.timeParameterizeTrajectory(
                points,
                config.getConstraints(),
                config.getStartVelocity(),
                config.getEndVelocity(),
                config.getMaxVelocity(),
                config.getMaxAcceleration(),
                config.getMaxAngularVelocity(),
                config.getMaxAngularAcceleration()
        );
    }

    public static OmniTrajectory generateTrajectory(List<OmniWaypoint> waypoints, OmniTrajectoryConfig config) {
        List<Pose2d> poses = new ArrayList<>();
        List<Rotation2d> pathHeadings = new ArrayList<>();
        waypoints.forEach((waypoint) -> {poses.add(waypoint.getPose()); pathHeadings.add(waypoint.getPathHeading());});
        return generateTrajectory(poses, pathHeadings, config);
    }

    /*
        public static OmniTrajectory generateTrajectory(ControlVectorList controlVectors, List<Pose2d> robotPoses, OmniTrajectoryConfig config) {
        final ArrayList<Spline.ControlVector> newControlVectors = new ArrayList<>(controlVectors.size());

        controlVectors.forEach((controlVector -> newControlVectors.add(new Spline.ControlVector(controlVector.x, controlVector.y))));

        ArrayList<Rotation2d> robotHeadings = new ArrayList<>();
        robotPoses.forEach(pose2d -> robotHeadings.add(pose2d.getRotation()));

        // Get the spline points
        List<PoseWithCurvatureAndRotation> points;
        try {
            points = splinePointsFromSplines(
                    SplineHelper.getQuinticSplinesFromControlVectors(newControlVectors.toArray(new Spline.ControlVector[]{})),
                    robotHeadings
            );
        } catch (SplineParameterizer.MalformedSplineException ex) {
            reportError(ex.getMessage(), ex.getStackTrace());
            return kDoNothingTrajectory;
        }

        
        return OmniTrajectoryParameterizer.timeParameterizeTrajectory(
                points,
                config.getConstraints(),
                config.getStartVelocity(),
                config.getEndVelocity(),
                config.getMaxVelocity(),
                config.getMaxAcceleration(),
                config.getMaxAngularVelocity(),
                config.getMaxAngularAcceleration()
        );
    }*/

    public static List<PoseWithCurvatureAndRotation> splinePointsFromSplines(Spline[] splines, List<Rotation2d> headings) {
        if (splines.length + 1 != headings.size()) {
            throw new IllegalArgumentException("Mismatch between number of splines and number of robot headings!");
        }

        // Create the vector of spline points.
        ArrayList<PoseWithCurvatureAndRotation> splinePoints = new ArrayList<>();

        // Add the first point to the vector.
        splinePoints.add(new PoseWithCurvatureAndRotation(splines[0].getPoint(0.0)));

        // Iterate through the vector and parameterize each spline, adding the
        // parameterized points to the final vector.
        for (int x = 0; x < splines.length; x++) {
            List<PoseWithCurvature> points = SplineParameterizer.parameterize(splines[x]);
            List<PoseWithCurvatureAndRotation> completePoints = new ArrayList<>();

            double rotationDuringSpline = headings.get(x + 1).minus(headings.get(x)).getRadians();
            double splineDistance = 0.0;

            for (int y = 0; y < points.size() - 1; y++) {
                splineDistance += points.get(y).poseMeters.getTranslation().getDistance(points.get(y + 1).poseMeters.getTranslation());
            }

            double rotationPerPoint = rotationDuringSpline / splineDistance;

            System.out.println(rotationDuringSpline);
            System.out.println(rotationPerPoint);

            for (PoseWithCurvature pose : points) completePoints.add(new PoseWithCurvatureAndRotation(pose, rotationPerPoint));

            if (x == 0) splinePoints.get(0).rotationRadiansPerMeter = rotationPerPoint;

            // Append the array of poses to the vector. We are removing the first
            // point because it's a duplicate of the last point from the previous
            // spline.
            splinePoints.addAll(completePoints.subList(1, completePoints.size()));
        }

        return splinePoints;
    }

    // Work around type erasure signatures
    /*public static class ControlVectorList extends ArrayList<Spline.ControlVector> {
        public ControlVectorList(int initialCapacity) {
            super(initialCapacity);
        }

        public ControlVectorList() {
            super();
        }

        public ControlVectorList(Collection<? extends Spline.ControlVector> collection) {
            super(collection);
        }
    }*/
}
