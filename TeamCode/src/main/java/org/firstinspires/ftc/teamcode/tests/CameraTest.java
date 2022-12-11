package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;

//@Disabled
@TeleOp(name = "Camera Test")
public class CameraTest extends LinearOpMode {
    private CameraSubsystem camera;

    @Override
    public void runOpMode() {
        this.camera = new CameraSubsystem(hardwareMap, telemetry);

        while (opModeInInit()) {
            int location = this.camera.getParkingLocation();
            telemetry.addData("Location: ", location);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            int location = this.camera.getParkingLocation();
            telemetry.addData("Location: ", location);
            telemetry.update();
        }
    }
}
