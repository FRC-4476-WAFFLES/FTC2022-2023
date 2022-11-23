package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class SignalSleevePipeline extends OpenCvPipeline {
    private final int THRESHOLD = 100;

    Telemetry telemetry;

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
        if (this.isFirstLocation(frame)) this.parkingLocation = 1;
        else if (this.isSecondLocation(frame)) this.parkingLocation = 2;
        else if (this.isThirdLocation(frame)) this.parkingLocation = 3;

        return frame;
    }

    private boolean isFirstLocation(Mat frame) {
        Mat hsv_image = new Mat();
        Imgproc.cvtColor(frame, hsv_image, Imgproc.COLOR_RGB2HSV);

        // Convert frame to a preferable format for detecting yellow
        Scalar lowHSV = new Scalar(0, 83, 190); // lower bound HSV for yellow
        Scalar highHSV = new Scalar(77, 205, 255); // higher bound HSV for yellow
        Mat thresh = new Mat();

        // Convert to black and white image, white region represents yellow
        Core.inRange(hsv_image, lowHSV, highHSV, thresh);

        double roiAverage = Core.mean(thresh.submat(SignalSleeveROI)).val[0];

        if (roiAverage > this.THRESHOLD) return true;
        return false;
    }

    private boolean isSecondLocation(Mat frame) {
        // Convert frame to a preferable format for detecting green
        Scalar lowHSV = new Scalar(44, 21, 96); // lower bound HSV for green
        Scalar highHSV = new Scalar(83, 94, 255); // higher bound HSV for green
        Mat thresh = new Mat();

        // Convert to black and white image, white region represents yellow
        Core.inRange(frame, lowHSV, highHSV, thresh);

        double roiAverage = Core.mean(thresh.submat(SignalSleeveROI)).val[0];

        if (roiAverage > this.THRESHOLD) return true;
        return false;
    }

    private boolean isThirdLocation(Mat frame) {
        Scalar lowRGB = new Scalar(0, 43, 112);
        Scalar highRGB = new Scalar(139, 126, 255);

        Mat image = new Mat();
        Imgproc.cvtColor(frame, image, Imgproc.COLOR_BGR2RGB);
        Core.inRange(image, lowRGB, highRGB, image);

        double roiAverage = Core.mean(image.submat(SignalSleeveROI)).val[0];

        if (roiAverage > this.THRESHOLD) return true;
        return false;
    }

    public int getParkingLocation() {
        return this.parkingLocation;
    }
}
