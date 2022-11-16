package lib.autoNavigation.trajectory;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

import lib.autoNavigation.math.MathUtil;

public class OmniTrajectory {
    private final double m_totalTimeSeconds;
    private final List<State> m_states;

    public OmniTrajectory(List<State> states) {
        this.m_states = states;
        this.m_totalTimeSeconds = states.get(states.size() - 1).timeSeconds;
    }

    /**
     * Returns the initial pose of the trajectory.
     *
     * @return The initial pose of the trajectory.
     */
    public Pose2d getInitialPose() {
        return sample(0).poseMeters;
    }

    /**
     * Returns the overall duration of the trajectory.
     *
     * @return The duration of the trajectory.
     */
    public double getTotalTimeSeconds() {
        return m_totalTimeSeconds;
    }

    /**
     * Return the states of the trajectory.
     *
     * @return The states of the trajectory.
     */
    public List<State> getStates() {
        return m_states;
    }

    /**
     * Sample the trajectory at a point in time.
     *
     * @param timeSeconds The point in time since the beginning of the trajectory to sample.
     * @return The state at that point in time.
     */
    public State sample(double timeSeconds) {
        if (timeSeconds <= m_states.get(0).timeSeconds) {
            return m_states.get(0);
        }
        if (timeSeconds >= m_totalTimeSeconds) {
            return m_states.get(m_states.size() - 1);
        }

        // To get the element that we want, we will use a binary search algorithm
        // instead of iterating over a for-loop. A binary search is O(std::log(n))
        // whereas searching using a loop is O(n).

        // This starts at 1 because we use the previous state later on for
        // interpolation.
        int low = 1;
        int high = m_states.size() - 1;

        while (low != high) {
            int mid = (low + high) / 2;
            if (m_states.get(mid).timeSeconds < timeSeconds) {
                // This index and everything under it are less than the requested
                // timestamp. Therefore, we can discard them.
                low = mid + 1;
            } else {
                // t is at least as large as the element at this index. This means that
                // anything after it cannot be what we are looking for.
                high = mid;
            }
        }

        // High and Low should be the same.

        // The sample's timestamp is now greater than or equal to the requested
        // timestamp. If it is greater, we need to interpolate between the
        // previous state and the current state to get the exact state that we
        // want.
        final State sample = m_states.get(low);
        final State prevSample = m_states.get(low - 1);

        // If the difference in states is negligible, then we are spot on!
        if (Math.abs(sample.timeSeconds - prevSample.timeSeconds) < 1E-9) {
            return sample;
        }
        // Interpolate between the two states for the state that we want.
        return prevSample.interpolate(sample,
                (timeSeconds - prevSample.timeSeconds) / (sample.timeSeconds - prevSample.timeSeconds));
    }

    @Override
    public String toString() {
        String stateList = m_states.stream().map(State::toString).collect(Collectors.joining(", \n"));
        return String.format("Trajectory - Seconds: %.2f, States:\n%s", m_totalTimeSeconds, stateList);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        OmniTrajectory that = (OmniTrajectory) o;
        return Double.compare(that.m_totalTimeSeconds, m_totalTimeSeconds) == 0 && m_states.equals(that.m_states);
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_totalTimeSeconds, m_states);
    }

    public static class State {
        // The time elapsed since the beginning of the trajectory.
        public double timeSeconds;

        // The speed at that point of the trajectory.
        public double velocityMetersPerSecond;

        // The acceleration at that point of the trajectory.
        public double accelerationMetersPerSecondSq;

        public double angularVelocityRadiansPerSecond;

        public double angularAccelerationRadiansPerSecondSq;

        // The pose at that point of the trajectory.
        public Pose2d poseMeters;

        public Rotation2d pathHeading;

        public double rotationRadPerMeter;

        // The curvature at that point of the trajectory.
        public double curvatureRadPerMeter;

        public State() {
            poseMeters = new Pose2d();
            pathHeading = new Rotation2d();
        }

        public State(
                double timeSeconds,
                double velocityMetersPerSecond,
                double accelerationMetersPerSecondSq,
                double angularVelocityRadiansPerSecond,
                double angularAccelerationRadiansPerSecondSq,
                Pose2d poseMeters,
                Rotation2d pathHeading,
                double rotationRadPerMeter,
                double curvatureRadPerMeter
        ) {
            this.timeSeconds = timeSeconds;
            this.velocityMetersPerSecond = velocityMetersPerSecond;
            this.accelerationMetersPerSecondSq = accelerationMetersPerSecondSq;
            this.angularVelocityRadiansPerSecond = angularVelocityRadiansPerSecond;
            this.angularAccelerationRadiansPerSecondSq = angularAccelerationRadiansPerSecondSq;
            this.poseMeters = poseMeters;
            this.pathHeading = pathHeading;
            this.rotationRadPerMeter = rotationRadPerMeter;
            this.curvatureRadPerMeter = curvatureRadPerMeter;
        }

        /**
         * Interpolates between two States.
         *
         * @param endValue The end value for the interpolation.
         * @param i        The interpolant (fraction).
         * @return The interpolated state.
         */

        @SuppressWarnings("ParameterName")
        public State interpolate(State endValue, double i) {
            // Find the new t value.
            final double newT = MathUtil.interpolate(timeSeconds, endValue.timeSeconds, i);

            // Find the delta time between the current state and the interpolated state.
            final double deltaT = newT - timeSeconds;

            // If delta time is negative, flip the order of interpolation.
            if (deltaT < 0) {
                return endValue.interpolate(this, 1 - i);
            }

            // Check whether the robot is reversing at this stage.
            final boolean reversing = velocityMetersPerSecond < 0
                    || Math.abs(velocityMetersPerSecond) < 1E-9 && accelerationMetersPerSecondSq < 0;

            // Calculate the new velocity
            // v_f = v_0 + at
            final double newV = velocityMetersPerSecond + (accelerationMetersPerSecondSq * deltaT);

            final double newThetaV = angularVelocityRadiansPerSecond + (angularAccelerationRadiansPerSecondSq * deltaT);

            // Calculate the change in position.
            // delta_s = v_0 t + 0.5 at^2
            final double newS = (velocityMetersPerSecond * deltaT
                    + 0.5 * accelerationMetersPerSecondSq * Math.pow(deltaT, 2)) * (reversing ? -1.0 : 1.0);

            // Return the new state. To find the new position for the new state, we need
            // to interpolate between the two endpoint poses. The fraction for
            // interpolation is the change in position (delta s) divided by the total
            // distance between the two endpoints.
            final double interpolationFrac = newS
                    / endValue.poseMeters.getTranslation().getDistance(poseMeters.getTranslation());

            return new State(
                    newT,
                    newV,
                    accelerationMetersPerSecondSq,
                    newThetaV,
                    angularAccelerationRadiansPerSecondSq,
                    MathUtil.interpolate(poseMeters, endValue.poseMeters, interpolationFrac),
                    MathUtil.interpolate(pathHeading, endValue.pathHeading, interpolationFrac),
                    MathUtil.interpolate(rotationRadPerMeter, endValue.curvatureRadPerMeter, interpolationFrac),
                    MathUtil.interpolate(curvatureRadPerMeter, endValue.curvatureRadPerMeter, interpolationFrac)
            );
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            State state = (State) o;
            return Double.compare(state.timeSeconds, timeSeconds) == 0
                    && Double.compare(state.velocityMetersPerSecond, velocityMetersPerSecond) == 0
                    && Double.compare(state.accelerationMetersPerSecondSq, accelerationMetersPerSecondSq) == 0
                    && Double.compare(state.angularVelocityRadiansPerSecond, angularVelocityRadiansPerSecond) == 0
                    && Double.compare(state.angularAccelerationRadiansPerSecondSq, angularAccelerationRadiansPerSecondSq) == 0
                    && Double.compare(state.curvatureRadPerMeter, curvatureRadPerMeter) == 0
                    && poseMeters.equals(state.poseMeters)
                    && pathHeading.equals(state.pathHeading);
        }

        @Override
        public int hashCode() {
            return Objects.hash(
                    timeSeconds,
                    velocityMetersPerSecond,
                    accelerationMetersPerSecondSq,
                    angularVelocityRadiansPerSecond,
                    angularAccelerationRadiansPerSecondSq,
                    poseMeters,
                    pathHeading,
                    curvatureRadPerMeter
            );
        }

        /*
        @Override
        public String toString() {
            return "State{" +
                    "timeSeconds=" + timeSeconds +
                    ", velocityMetersPerSecond=" + velocityMetersPerSecond +
                    ", accelerationMetersPerSecondSq=" + accelerationMetersPerSecondSq +
                    ", angularVelocityRadPerSecond=" + angularVelocityRadiansPerSecond +
                    ", angularAccelerationRadPerSecondSq=" + angularAccelerationRadiansPerSecondSq +
                    ", poseMeters=" + poseMeters +
                    ", pathHeading=" + pathHeading +
                    ", curvatureRadPerMeter=" + curvatureRadPerMeter +
                    '}';
        }*/

        @Override
        public String toString() {
            return String.format(
                "State(Sec: %.2f, Vel m/s: %.2f, Accel m/s/s: %.2f, Angle Vel r/s: %.2f, Angle Accel r/s/s: %.2f, Pose: %s, Path Heading: %s, Rotation Per Meter: %.2f, Curvature: %.2f)",
                timeSeconds,
                velocityMetersPerSecond,
                accelerationMetersPerSecondSq,
                angularVelocityRadiansPerSecond,
                angularAccelerationRadiansPerSecondSq,
                poseMeters,
                pathHeading,
                rotationRadPerMeter,
                curvatureRadPerMeter
            );
        }
    }
}
