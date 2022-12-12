package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

import lib.autoNavigation.math.MathUtil;

public class ArmJoystickControl extends CommandBase {
    private final DoubleSupplier joystickValue;

    public ArmJoystickControl(DoubleSupplier joystickValue) {
        this.joystickValue = joystickValue;
        addRequirements(ArmSubsystem.getInstance());
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double value = MathUtil.applyDeadband(joystickValue.getAsDouble(), 0.05);
        ArmSubsystem.getInstance().moveTargetPositionWithJoystick(value);
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
