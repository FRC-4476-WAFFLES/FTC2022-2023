package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class SignalSleevePipeline extends OpenCvPipeline {
    Telemetry telemetry;

    Mat image = new Mat();

    private int parkingLocation;

    public Rect SignalSleeveROI = new Rect(
            new Point(100, 0),
            new Point(300, 300)
    );

    public SignalSleevePipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.parkingLocation = 0;
    }

    @Override
    public Mat processFrame(Mat frame) {
        // Find parking location

        return frame;
    }

    public int getParkingLocation() {
        return this.parkingLocation;
    }
}
