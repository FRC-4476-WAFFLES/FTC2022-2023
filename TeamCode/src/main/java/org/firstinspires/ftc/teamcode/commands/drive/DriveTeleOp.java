package org.firstinspires.ftc.teamcode.commands.drive;

import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.MAX_ROT_SPEED_RAD_PER_SEC;
import static org.firstinspires.ftc.teamcode.Constants.DriveConstants.MAX_SPEED_M_PER_S;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

import lib.autoNavigation.math.MathUtil;

public class DriveTeleOp extends CommandBase {
    private final DoubleSupplier forwardSupplier, rightSupplier, rotationSupplier;

    public DriveTeleOp(
            DoubleSupplier forwardSupplier,
            DoubleSupplier rightSupplier,
            DoubleSupplier rotationSupplier
    ) {
        this.forwardSupplier = forwardSupplier;
        this.rightSupplier = rightSupplier;
        this.rotationSupplier = rotationSupplier;
        addRequirements(DriveSubsystem.getInstance());
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double forward = MathUtil.applyDeadband(forwardSupplier.getAsDouble(), 0.05);
        double right = MathUtil.applyDeadband(rightSupplier.getAsDouble(), 0.05);
        double rotation = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), 0.05);

        forward *= MAX_SPEED_M_PER_S;
        right *= MAX_SPEED_M_PER_S;
        rotation *= MAX_ROT_SPEED_RAD_PER_SEC;

        DriveSubsystem.getInstance().driveTeleOp(
                forward,
                right,
                rotation,
                true
        );
    }

    @Override
    public void end(boolean interrupted) {
        DriveSubsystem.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
