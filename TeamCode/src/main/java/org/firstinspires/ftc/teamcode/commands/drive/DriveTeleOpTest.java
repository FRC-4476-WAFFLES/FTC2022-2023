package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

import lib.autoNavigation.math.MathUtil;

public class DriveTeleOpTest extends CommandBase {
    private final DoubleSupplier forwardSupplier, rightSupplier, rotationSupplier;
    private final Telemetry telemetry;

    public DriveTeleOpTest(
            DoubleSupplier forwardSupplier,
            DoubleSupplier rightSupplier,
            DoubleSupplier rotationSupplier,
            Telemetry telemetry
    ) {
        this.forwardSupplier = forwardSupplier;
        this.rightSupplier = rightSupplier;
        this.rotationSupplier = rotationSupplier;
        this.telemetry = telemetry;
        addRequirements(DriveSubsystem.getInstance());
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double forward = MathUtil.applyDeadband(forwardSupplier.getAsDouble(), 0.05);
        double right = MathUtil.applyDeadband(rightSupplier.getAsDouble(), 0.05);
        double rotation = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), 0.05);

        forward *= 1.5;
        right *= 1.5;
        rotation *= 2.0;

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
