package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.AlternativePipeline;
import org.firstinspires.ftc.teamcode.pipelines.ParkingLocationPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import lib.autoNavigation.math.MathUtil;

public class CameraSubsystem extends SubsystemBase {
    private static final CameraSubsystem instance = new CameraSubsystem();

    private final ElapsedTime timer = new ElapsedTime();

    private final int[] parkingLocations = new int[10];

    private boolean isReady = false;

    OpenCvCamera webcam;
    int cameraMonitorViewId;

    ParkingLocationPipeline pipeline;

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
                isReady = true;
            }

            @Override
            public void onError(int errorCode) {
                // Camera fails to be opened
                telemetry.addData("Failed to open webcam. Error code:", errorCode);
                telemetry.update();
            }
        });
    }

    @Override
    public void periodic() {
        if (isReady && timer.time() > 0.1) {
            System.arraycopy(parkingLocations, 0, parkingLocations, 1, parkingLocations.length - 2);
            parkingLocations[0] = getParkingLocation();
            timer.reset();
        }
    }

    public int getParkingLocation() {
        return this.pipeline.getParkingLocation();
    }

    public int getFilteredParkingLocation() {
        return MathUtil.mode(parkingLocations);
    }

    public void stop() {
        this.webcam.stopStreaming();
    }

    public boolean isReady() {
        return isReady;
    }
}
