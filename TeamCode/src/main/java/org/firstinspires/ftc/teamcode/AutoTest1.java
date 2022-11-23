package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class AutoTest1 extends LinearOpMode {
    DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);

    @Override
    public void runOpMode() {
        while (opModeIsActive()) {
            driveSubsystem.driveAuto();
            driveSubsystem.periodic();
        }

        driveSubsystem.stop();
    }
}
