package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class CameraSubsystem extends SubsystemBase {
    private static final CameraSubsystem instance = new CameraSubsystem();

    OpenCvCamera webcam;
    int cameraMonitorViewId;

    AlternativePipeline pipeline;

    private Telemetry telemetry;

    private CameraSubsystem() {}

    public static CameraSubsystem getInstance() {
        return instance;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        // Get webcam object from hardwareMap
        this.cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        this.webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(
                        WebcamName.class, "Webcam 1"
                ), this.cameraMonitorViewId);

        // Set pipeline to Freight Detector
        this.pipeline = new AlternativePipeline();
        this.webcam.setPipeline(pipeline);

        // Activate camera
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming from the camera
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                telemetry.addLine("Opened webcam.");
            }

            @Override
            public void onError(int errorCode) {
                // Camera fails to be opened
                telemetry.addData("Failed to open webcam. Error code:", errorCode);
                telemetry.update();
            }
        });
    }

    public int getParkingLocation() {
        return this.pipeline.getParkingLocation();
    }

    public void stop() {
        this.webcam.stopStreaming();
    }
}
