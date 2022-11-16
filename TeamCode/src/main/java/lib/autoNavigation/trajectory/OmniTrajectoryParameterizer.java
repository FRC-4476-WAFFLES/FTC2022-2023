package lib.autoNavigation.trajectory;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import java.util.ArrayList;
import java.util.List;

import lib.autoNavigation.trajectory.constraint.OmniTrajectoryConstraint;
import lib.autoNavigation.spline.PoseWithCurvatureAndRotation;

public final class OmniTrajectoryParameterizer {
    /**
     * Private constructor because this is a utility class.
     */
    private OmniTrajectoryParameterizer() {}

    public static OmniTrajectory timeParameterizeTrajectory(
            List<PoseWithCurvatureAndRotation> points,
            List<OmniTrajectoryConstraint> constraints,
            double startVelocityMetersPerSecond,
            double endVelocityMetersPerSecond,
            double maxVelocityMetersPerSecond,
            double maxAccelerationMetersPerSecondSq,
            double maxAngularVelocityRadsPerSecond,
            double maxAngularAccelerationRadsPerSecondSq
    ) {
        ArrayList<ConstrainedState> constrainedStates = new ArrayList<>(points.size());
        ConstrainedState predecessor = new ConstrainedState(
                points.get(0),
                0.0,
                startVelocityMetersPerSecond,
                -maxAccelerationMetersPerSecondSq,
                maxAccelerationMetersPerSecondSq,
                0.0,
                -maxAngularAccelerationRadsPerSecondSq,
                maxAngularAccelerationRadsPerSecondSq
        );

        // Forward pass
        for (int i = 0; i < points.size(); i++) {
            constrainedStates.add(new ConstrainedState());
            ConstrainedState constrainedState = constrainedStates.get(i);
            constrainedState.pose = points.get(i);

            // Begin constraining based on predecessor.
            double ds = constrainedState.pose.poseMeters.getTranslation().getDistance(predecessor.pose.poseMeters.getTranslation());
            constrainedState.distanceMeters = predecessor.distanceMeters + ds;

            // We may need to iterate to find the maximum end velocity and common
            // acceleration, since acceleration limits may be a function of velocity.
            while (true) {
                // Enforce global max velocity and max reachable velocity by global
                // acceleration limit. vf = std::sqrt(vi^2 + 2*a*d).
                constrainedState.maxVelocityMetersPerSecond = Math.min(
                        maxVelocityMetersPerSecond,
                        Math.sqrt(Math.pow(predecessor.maxVelocityMetersPerSecond, 2) + predecessor.maxAccelerationMetersPerSecondSq * ds * 2.0)
                );

                constrainedState.maxAngularVelocityRadiansPerSecond = Math.min(
                        maxAngularVelocityRadsPerSecond,
                        Math.sqrt(Math.pow(predecessor.maxAngularVelocityRadiansPerSecond, 2) + predecessor.maxAngularAccelerationRadiansPerSecondSq * ds * 2)
                );

                constrainedState.minAccelerationMetersPerSecondSq = -maxAccelerationMetersPerSecondSq;
                constrainedState.maxAccelerationMetersPerSecondSq = maxAccelerationMetersPerSecondSq;

                constrainedState.minAngularAccelerationRadiansPerSecondSq = -maxAngularAccelerationRadsPerSecondSq;
                constrainedState.maxAngularAccelerationRadiansPerSecondSq = maxAngularAccelerationRadsPerSecondSq;

                // At this point, the constrained state is fully constructed apart from
                // all the custom-defined user constraints.
                for (final OmniTrajectoryConstraint constraint : constraints) {
                    constrainedState.maxVelocityMetersPerSecond = Math.min(
                            constrainedState.maxVelocityMetersPerSecond,
                            constraint.getMaxVelocityMetersPerSecond(
                                    constrainedState.pose.poseMeters,
                                    constrainedState.pose.curvatureRadPerMeter,
                                    0,
                                    constrainedState.maxVelocityMetersPerSecond
                            )
                    );
                }

                // Now enforce all acceleration limits.
                enforceAccelerationLimits(constraints, constrainedState);

                double actualAcceleration = (Math.pow(constrainedState.maxVelocityMetersPerSecond, 2)
                    - Math.pow(predecessor.maxVelocityMetersPerSecond, 2))
                    / (ds * 2.0);

                double angularAcceleration = actualAcceleration * constrainedState.pose.rotationRadiansPerMeter;

                if (angularAcceleration > constrainedState.maxAngularAccelerationRadiansPerSecondSq) {
                    double ratio = constrainedState.maxAngularAccelerationRadiansPerSecondSq / angularAcceleration;
                    constrainedState.maxAccelerationMetersPerSecondSq *= ratio;
                }

                double angularVelocity = constrainedState.maxVelocityMetersPerSecond * constrainedState.pose.rotationRadiansPerMeter;

                if (angularVelocity > constrainedState.maxAngularVelocityRadiansPerSecond) {
                    double ratio = constrainedState.maxAngularVelocityRadiansPerSecond / angularVelocity;
                    constrainedState.maxVelocityMetersPerSecond *= ratio;
                }

                if (ds < 1E-6) {
                    break;
                }

                // If the actual acceleration for this state is higher than the max
                // acceleration that we applied, then we need to reduce the max
                // acceleration of the predecessor and try again.
                

                // If we violate the max acceleration constraint, let's modify the
                // predecessor.
                if (constrainedState.maxAccelerationMetersPerSecondSq < actualAcceleration - 1E-6) {
                    predecessor.maxAccelerationMetersPerSecondSq = constrainedState.maxAccelerationMetersPerSecondSq;
                } else {
                    // Constrain the predecessor's max acceleration to the current
                    // acceleration.
                    if (actualAcceleration > predecessor.minAccelerationMetersPerSecondSq) {
                        predecessor.maxAccelerationMetersPerSecondSq = actualAcceleration;
                    }
                    // If the actual acceleration is less than the predecessor's min
                    // acceleration, it will be repaired in the backward pass.
                    break;
                }
            }
            predecessor = constrainedState;
        }

        ConstrainedState successor = new ConstrainedState(
                points.get(points.size() - 1),
                constrainedStates.get(constrainedStates.size() - 1).distanceMeters,
                endVelocityMetersPerSecond,
                -maxAccelerationMetersPerSecondSq,
                maxAccelerationMetersPerSecondSq,
                0,
                -maxAngularAccelerationRadsPerSecondSq,
                maxAngularAccelerationRadsPerSecondSq
        );

        // Backward pass
        for (int i = points.size() - 1; i >= 0; i--) {
            ConstrainedState constrainedState = constrainedStates.get(i);
            double ds = constrainedState.distanceMeters - successor.distanceMeters; // negative

            while (true) {
                // Enforce max velocity limit (reverse)
                // vf = std::sqrt(vi^2 + 2*a*d), where vi = successor.
                double newMaxVelocity = Math.sqrt(Math.pow(successor.maxVelocityMetersPerSecond, 2) + successor.minAccelerationMetersPerSecondSq * ds * 2.0);

                double angularAcceleration = (Math.pow(constrainedState.maxVelocityMetersPerSecond * constrainedState.pose.rotationRadiansPerMeter, 2)
                        - Math.pow(successor.maxVelocityMetersPerSecond * successor.pose.rotationRadiansPerMeter, 2))
                        / (ds * constrainedState.pose.rotationRadiansPerMeter * 2.0);

                if (angularAcceleration < constrainedState.minAngularAccelerationRadiansPerSecondSq) {
                    double ratio = constrainedState.maxAngularAccelerationRadiansPerSecondSq / angularAcceleration;
                    constrainedState.maxAccelerationMetersPerSecondSq *= ratio;
                }

                double angularVelocity = constrainedState.maxVelocityMetersPerSecond * constrainedState.pose.rotationRadiansPerMeter;

                if (angularVelocity < -constrainedState.maxAngularVelocityRadiansPerSecond) {
                    double ratio = constrainedState.maxAngularVelocityRadiansPerSecond / angularVelocity;
                    constrainedState.maxVelocityMetersPerSecond *= Math.abs(ratio);
                }

                // No more limits to impose! This state can be finalized.
                if (newMaxVelocity >= constrainedState.maxVelocityMetersPerSecond) {
                    break;
                }

                constrainedState.maxVelocityMetersPerSecond = newMaxVelocity;

                // Check all acceleration constraints with the new max velocity.
                enforceAccelerationLimits(constraints, constrainedState);

                if (ds > -1E-6) {
                    break;
                }

                // If the actual acceleration for this state is lower than the min
                // acceleration, then we need to lower the min acceleration of the
                // successor and try again.
                double actualAcceleration = (constrainedState.maxVelocityMetersPerSecond
                        * constrainedState.maxVelocityMetersPerSecond
                        - successor.maxVelocityMetersPerSecond * successor.maxVelocityMetersPerSecond)
                        / (ds * 2.0);

                if (constrainedState.minAccelerationMetersPerSecondSq > actualAcceleration + 1E-6) {
                    successor.minAccelerationMetersPerSecondSq
                            = constrainedState.minAccelerationMetersPerSecondSq;
                } else {
                    successor.minAccelerationMetersPerSecondSq = actualAcceleration;
                    break;
                }
            }
            successor = constrainedState;
        }

        // Now we can integrate the constrained states forward in time to obtain our
        // trajectory states.
        ArrayList<OmniTrajectory.State> states = new ArrayList<>(points.size());
        double timeSeconds = 0.0;
        double distanceMeters = 0.0;
        double velocityMetersPerSecond = 0.0;
        double angularVelocityRadiansPerSecond;

        Rotation2d heading = new Rotation2d();

        for (int i = 0; i < constrainedStates.size(); i++) {
            final ConstrainedState state = constrainedStates.get(i);

            // Calculate the change in position between the current state and the previous
            // state.
            double ds = state.distanceMeters - distanceMeters;

            double rs = ds * state.pose.rotationRadiansPerMeter;

            // Calculate the acceleration between the current state and the previous
            // state.
            double accel = (state.maxVelocityMetersPerSecond * state.maxVelocityMetersPerSecond
                    - velocityMetersPerSecond * velocityMetersPerSecond) / (ds * 2.0);

            double angularAccel;

            double calculatedAngularAccel = (Math.pow(state.maxVelocityMetersPerSecond * state.pose.rotationRadiansPerMeter, 2)
                    - Math.pow(velocityMetersPerSecond * state.pose.rotationRadiansPerMeter, 2)) / (rs * 2.0);

            if (calculatedAngularAccel > state.maxAngularAccelerationRadiansPerSecondSq) {
                accel *= (state.maxAngularAccelerationRadiansPerSecondSq / calculatedAngularAccel);
                angularAccel = state.maxAngularAccelerationRadiansPerSecondSq;
            } else if (calculatedAngularAccel < state.minAngularAccelerationRadiansPerSecondSq) {
                accel *= (state.minAngularAccelerationRadiansPerSecondSq / calculatedAngularAccel);
                angularAccel = state.minAngularAccelerationRadiansPerSecondSq;
            } else {
                angularAccel = calculatedAngularAccel;
            }

            // Calculate dt
            double dt = 0.0;
            if (i > 0) {
                states.get(i - 1).accelerationMetersPerSecondSq = accel;
                states.get(i - 1).angularAccelerationRadiansPerSecondSq = angularAccel;
                if (Math.abs(accel) > 1E-6) {
                    // v_f = v_0 + a * t
                    dt = (state.maxVelocityMetersPerSecond - velocityMetersPerSecond) / accel;
                } else if (Math.abs(velocityMetersPerSecond) > 1E-6) {
                    // delta_x = v * t
                    dt = ds / velocityMetersPerSecond;
                } else {
                    throw new RuntimeException("Something went wrong");
                }
            }

            velocityMetersPerSecond = state.maxVelocityMetersPerSecond;
            angularVelocityRadiansPerSecond = velocityMetersPerSecond * state.pose.rotationRadiansPerMeter;
            distanceMeters = state.distanceMeters;

            timeSeconds += dt;

            heading = new Rotation2d(heading.getRadians() + rs);

            states.add(new OmniTrajectory.State(
                    timeSeconds,
                    velocityMetersPerSecond,
                    accel,
                    angularVelocityRadiansPerSecond,
                    angularAccel,
                    new Pose2d(state.pose.poseMeters.getTranslation(), new Rotation2d(heading.getRadians())),
                    new Rotation2d(state.pose.poseMeters.getRotation().getRadians()),
                    state.pose.rotationRadiansPerMeter,
                    state.pose.curvatureRadPerMeter
            ));
        }

        return new OmniTrajectory(states);
    }

    private static void enforceAccelerationLimits(
            List<OmniTrajectoryConstraint> constraints,
            ConstrainedState state
    ) {
        for (final OmniTrajectoryConstraint constraint : constraints) {
            final OmniTrajectoryConstraint.MinMax minMaxAccel = constraint.getMinMaxAccelerationMetersPerSecondSq(
                    state.pose.poseMeters,
                    state.pose.curvatureRadPerMeter,
                    0,
                    state.maxVelocityMetersPerSecond
            );

            state.minAccelerationMetersPerSecondSq = Math.max(
                    state.minAccelerationMetersPerSecondSq,
                    minMaxAccel.minAccelerationMetersPerSecondSq
            );

            state.maxAccelerationMetersPerSecondSq = Math.min(
                    state.maxAccelerationMetersPerSecondSq,
                    minMaxAccel.maxAccelerationMetersPerSecondSq
            );
        }
    }

    private static class ConstrainedState {
        PoseWithCurvatureAndRotation pose;
        double distanceMeters;
        double maxVelocityMetersPerSecond;
        double minAccelerationMetersPerSecondSq;
        double maxAccelerationMetersPerSecondSq;
        double maxAngularVelocityRadiansPerSecond;
        double minAngularAccelerationRadiansPerSecondSq;
        double maxAngularAccelerationRadiansPerSecondSq;

        ConstrainedState(
                PoseWithCurvatureAndRotation pose,
                double distanceMeters,
                double maxVelocityMetersPerSecond,
                double minAccelerationMetersPerSecondSq,
                double maxAccelerationMetersPerSecondSq,
                double maxAngularVelocityRadiansPerSecond,
                double minAngularAccelerationRadiansPerSecondSq,
                double maxAngularAccelerationRadiansPerSecondSq
        ) {
            this.pose = pose;
            this.distanceMeters = distanceMeters;
            this.maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
            this.minAccelerationMetersPerSecondSq = minAccelerationMetersPerSecondSq;
            this.maxAccelerationMetersPerSecondSq = maxAccelerationMetersPerSecondSq;
            this.maxAngularVelocityRadiansPerSecond = maxAngularVelocityRadiansPerSecond;
            this.minAngularAccelerationRadiansPerSecondSq = minAngularAccelerationRadiansPerSecondSq;
            this.maxAngularAccelerationRadiansPerSecondSq = maxAngularAccelerationRadiansPerSecondSq;
        }

        ConstrainedState() {
            pose = new PoseWithCurvatureAndRotation();
        }
    }

    /*
        // Old/bad/useless code that I'm keeping until I get the other code actually working

        // break constrained states list into sublists based on where a change in angular direction is required.
        List<List<ConstrainedFullState>> sublists = new ArrayList<>();
        List<ConstrainedFullState> workingList = new ArrayList<>();
        int searchStart = 1;

        // Add the first state to the working list
        ConstrainedAngleState startConstrainedAngleState = new ConstrainedAngleState();
        startConstrainedAngleState.robotHeading = headingPoses.get(0).getRotation();
        workingList.add(new ConstrainedFullState(constrainedPositionStates.get(0), startConstrainedAngleState));

        boolean previousTurnIsClockwise = headingPoses.get(1).getRotation().minus(headingPoses.get(0).getRotation()).getRadians() > 0;

        for (int x = 1; x < headingPoses.size(); x++) {
            for (int i = searchStart; i < constrainedPositionStates.size(); i++) {
                // Store the pose2d object for heading for easy access
                Pose2d currentHeadingPose = headingPoses.get(x);

                // If the poses match, add the pose to the working sublist
                if (currentHeadingPose.getTranslation().equals(constrainedPositionStates.get(i).pose.poseMeters.getTranslation())) {
                    // Get the current turn direction
                    boolean currentTurnIsClockwise = currentHeadingPose.getRotation().minus(headingPoses.get(x - 1).getRotation()).getRadians() > 0;

                    // If the current turn direction is different from the previous direction,
                    // add the current working list to the list of sublists and clear the working list
                    if (currentTurnIsClockwise != previousTurnIsClockwise) {
                        sublists.add(workingList);
                        workingList.clear();
                        previousTurnIsClockwise = !previousTurnIsClockwise;
                    }

                    // Add a new constrained angle state with the heading to the working list
                    ConstrainedAngleState constrainedAngleState = new ConstrainedAngleState();
                    constrainedAngleState.robotHeading = headingPoses.get(x).getRotation();
                    workingList.add(new ConstrainedFullState(constrainedPositionStates.get(i), constrainedAngleState));

                    searchStart = i + 1;
                    break;
                }
            }
        }

        for (List<ConstrainedFullState> constrainedFullStateList : sublists) {

            for (int i = 0; i < constrainedFullStateList.size(); i++) {

            }
        }


        for rotation stuff:
        1. do max accel pass through all points - ignore max speed. If points result in motion in same direction, continue motion - don't stop. only have 0 angular velocity at points where chance of direction is required.
        2. combine position and rotation states to form full states.
        3. integrate states
        4. if max angular speed is exceeded, slow down position velocity
        5. allow for multiple iterations of loop to ensure position and heading states match up (optimize later)
        6. return a new OmniTrajectory of states
        7. profit

        return new OmniTrajectory(Collections.singletonList(new OmniTrajectory.State())); */
}
