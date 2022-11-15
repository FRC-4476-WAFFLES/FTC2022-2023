package lib.trajectory;


import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;

import java.util.ArrayList;
import java.util.List;

import lib.trajectory.constraint.OmniMecanumDriveKinematicsConstraint;
import lib.trajectory.constraint.OmniTrajectoryConstraint;

/**
 * Represents the configuration for generating a trajectory. This class stores the start velocity,
 * end velocity, max velocity, max acceleration, max angular velocity, max angular acceleration,
 * and custom constraints.
 *
 * <p>The class must be constructed with a max velocity, max acceleration, max angular velocity,
 * and max angular acceleration. The other parameters (start velocity, end velocity, constraints)
 * have been defaulted to reasonable values (0, 0, {}).
 * These values can be changed via the setXXX methods.
 */
public class OmniTrajectoryConfig {
    private final double m_maxVelocity;
    private final double m_maxAcceleration;
    private final double m_maxAngularVelocity;
    private final double m_maxAngularAcceleration;
    private final List<OmniTrajectoryConstraint> m_constraints;
    private double m_startVelocity;
    private double m_endVelocity;

    /**
     * Constructs the trajectory configuration class.
     *
     * @param maxVelocity The max velocity for the trajectory.
     * @param maxAcceleration The max acceleration for the trajectory.
     * @param maxAngularVelocity The max angular velocity for the trajectory.
     * @param maxAngularAcceleration The max angular acceleration for the trajectory.
     */
    public OmniTrajectoryConfig(
            double maxVelocity,
            double maxAcceleration,
            double maxAngularVelocity,
            double maxAngularAcceleration
    ) {
        this.m_maxVelocity = maxVelocity;
        this.m_maxAcceleration = maxAcceleration;
        this.m_maxAngularVelocity = maxAngularVelocity;
        this.m_maxAngularAcceleration = maxAngularAcceleration;
        this.m_constraints = new ArrayList<>();
        this.m_startVelocity = 0.0;
        this.m_endVelocity = 0.0;
    }

    /**
     * Adds a user-defined constraint to the trajectory.
     *
     * @param constraint The user-defined constraint.
     * @return Instance of the current config object.
     */
    public OmniTrajectoryConfig addConstraint(OmniTrajectoryConstraint constraint) {
        m_constraints.add(constraint);
        return this;
    }

    /**
     * Adds all user-defined constraints from a list to the trajectory.
     *
     * @param constraints List of user-defined constraints.
     * @return Instance of the current config object.
     */
    public OmniTrajectoryConfig addConstraints(List<? extends OmniTrajectoryConstraint> constraints) {
        this.m_constraints.addAll(constraints);
        return this;
    }

    /**
     * Adds a mecanum drive kinematics constraint to ensure that
     * no wheel velocity of a mecanum drive goes above the max velocity.
     *
     * @param kinematics The mecanum drive kinematics.
     * @return Instance of the current config object.
     */
    public OmniTrajectoryConfig setKinematics(MecanumDriveKinematics kinematics) {
        addConstraint(new OmniMecanumDriveKinematicsConstraint(kinematics, m_maxVelocity));
        return this;
    }

    /**
     * Returns the starting velocity of the trajectory.
     *
     * @return The starting velocity of the trajectory.
     */
    public double getStartVelocity() {
        return m_startVelocity;
    }

    /**
     * Sets the start velocity of the trajectory.
     *
     * @param startVelocityMetersPerSecond The start velocity of the trajectory.
     * @return Instance of the current config object.
     */
    public OmniTrajectoryConfig setStartVelocity(double startVelocityMetersPerSecond) {
        this.m_startVelocity = startVelocityMetersPerSecond;
        return this;
    }

    /**
     * Returns the ending velocity of the trajectory.
     *
     * @return The ending velocity of the trajectory.
     */
    public double getEndVelocity() {
        return m_endVelocity;
    }

    /**
     * Sets the end velocity of the trajectory.
     *
     * @param endVelocityMetersPerSecond The end velocity of the trajectory.
     * @return Instance of the current config object.
     */
    public OmniTrajectoryConfig setEndVelocity(double endVelocityMetersPerSecond) {
        this.m_endVelocity = endVelocityMetersPerSecond;
        return this;
    }

    /**
     * Returns the maximum velocity of the trajectory.
     *
     * @return The maximum velocity of the trajectory.
     */
    public double getMaxVelocity() {
        return m_maxVelocity;
    }

    /**
     * Returns the maximum acceleration of the trajectory.
     *
     * @return The maximum acceleration of the trajectory.
     */
    public double getMaxAcceleration() {
        return m_maxAcceleration;
    }

    /**
     * Returns the maximum angular velocity of the trajectory.
     *
     * @return The maximum angular velocity of the trajectory.
     */
    public double getMaxAngularVelocity() {
        return m_maxAngularVelocity;
    }

    /**
     * Returns the maximum angular acceleration of the trajectory.
     *
     * @return The maximum angular acceleration of the trajectory.
     */
    public double getMaxAngularAcceleration() {
        return m_maxAngularAcceleration;
    }

    /**
     * Returns the user-defined constraints of the trajectory.
     *
     * @return The user-defined constraints of the trajectory.
     */
    public List<OmniTrajectoryConstraint> getConstraints() {
        return m_constraints;
    }
}
