package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import lib.autoNavigation.command.MecanumControllerCommand;
import lib.autoNavigation.trajectory.OmniTrajectory;

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
                (chassisSpeeds -> DriveSubsystem.getInstance().setChassisSpeeds(chassisSpeeds))
        );
    }
}
