package lib.autoNavigation.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Consumer;
import java.util.function.Supplier;

import lib.autoNavigation.controller.HolonomicDriveController;
import lib.autoNavigation.trajectory.OmniTrajectory;

public class MecanumControllerCommand extends CommandBase {
    private final ElapsedTime m_timer = new ElapsedTime();
    private final OmniTrajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final HolonomicDriveController m_controller;
    private final Consumer<ChassisSpeeds> m_outputChassisSpeeds;

    public MecanumControllerCommand(
            OmniTrajectory trajectory,
            Supplier<Pose2d> pose,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            Subsystem... requirements
    ) {
        m_trajectory = trajectory;
        m_pose = pose;

        m_controller = new HolonomicDriveController(xController, yController, thetaController);

        m_outputChassisSpeeds = outputChassisSpeeds;

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        m_timer.reset();
    }

    @Override
    public void execute() {
        double curTime = m_timer.time();
        OmniTrajectory.State desiredState = m_trajectory.sample(curTime);

        ChassisSpeeds targetChassisSpeeds = m_controller.calculate(
                m_pose.get(),
                desiredState,
                desiredState.poseMeters.getRotation()
        );

        m_outputChassisSpeeds.accept(targetChassisSpeeds);
    }

    @Override
    public boolean isFinished() {
        return m_timer.time() >= m_trajectory.getTotalTimeSeconds();
    }
}
