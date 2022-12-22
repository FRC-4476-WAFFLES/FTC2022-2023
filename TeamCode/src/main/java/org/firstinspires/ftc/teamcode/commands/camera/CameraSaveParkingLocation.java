package org.firstinspires.ftc.teamcode.commands.camera;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;

public class CameraSaveParkingLocation extends CommandBase {
    private final ElapsedTime timer = new ElapsedTime();
    private double previousTime = 0;

    public CameraSaveParkingLocation() {
        addRequirements(CameraSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        if (CameraSubsystem.getInstance().isReady()) {

        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true; // Because this command just uses the webcam, this can safely run while disabled
    }
}
