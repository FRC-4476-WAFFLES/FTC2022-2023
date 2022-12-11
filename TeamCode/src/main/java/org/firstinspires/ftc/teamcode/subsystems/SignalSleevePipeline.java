package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SignalSleevePipeline extends OpenCvPipeline {
    private final int THRESHOLD = 100;

    Telemetry telemetry;

    private int parkingLocation;
    private Mat image;

    private Scalar firstLowHSV;
    private Scalar firstHighHSV;
    private Scalar secondLowHSV;
    private Scalar secondHighHSV;
    private Scalar thirdLowRGB;
    private Scalar thirdHighRGB;

    private double roiAverage;

    public Rect SignalSleeveROI = new Rect(
            new Point(900, 200),
            new Point(1200, 700)
    );

    public SignalSleevePipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.parkingLocation = 0;
        this.image = new Mat();

        this.firstLowHSV = new Scalar(0, 83, 190); // lower bound HSV for yellow
        this.firstHighHSV = new Scalar(77, 205, 255); // higher bound HSV for yellow

        this.secondLowHSV = new Scalar(44, 21, 96); // lower bound HSV for green
        this.secondHighHSV = new Scalar(83, 94, 255); // higher bound HSV for green

        this.thirdLowRGB = new Scalar(0, 43, 112);
        this.thirdHighRGB = new Scalar(139, 126, 255);
    }

    @Override
    public Mat processFrame(Mat frame) {
        // Find parking location

        if (this.isFirstLocation(frame)) this.parkingLocation = 1;
        // else if (this.isSecondLocation(frame)) this.parkingLocation = 2;
        // else if (this.isThirdLocation(frame)) this.parkingLocation = 3;
        else this.parkingLocation = 0;

        Imgproc.rectangle(this.image, this.SignalSleeveROI, new Scalar(0, 255, 0), 4);

        return this.image;
    }

    private boolean isFirstLocation(Mat frame) {
        /*
        Imgproc.cvtColor(frame, this.image, Imgproc.COLOR_RGB2HSV);

        // Convert to black and white image, white region represents yellow
        Core.inRange(this.image, this.firstLowHSV, this.firstHighHSV, this.image);

        this.roiAverage = Core.mean(this.image.submat(SignalSleeveROI)).val[0];

        if (this.roiAverage > this.THRESHOLD) return true;
        return false; */
        Imgproc.cvtColor(frame, this.image, Imgproc.COLOR_BGR2RGB);
        Core.inRange(this.image, this.thirdLowRGB, this.thirdHighRGB, this.image);

        this.roiAverage = Core.mean(this.image.submat(SignalSleeveROI)).val[0];

        if (this.roiAverage > this.THRESHOLD) return true;
        return false;
    }

    private boolean isSecondLocation(Mat frame) {
        // Convert to black and white image, white region represents yellow
        Core.inRange(frame, this.secondLowHSV, this.secondHighHSV, this.image);

        this.roiAverage = Core.mean(this.image.submat(SignalSleeveROI)).val[0];

        if (this.roiAverage > this.THRESHOLD) return true;
        return false;
    }

    private boolean isThirdLocation(Mat frame) {
        Imgproc.cvtColor(frame, this.image, Imgproc.COLOR_BGR2RGB);
        Core.inRange(this.image, this.thirdLowRGB, this.thirdHighRGB, this.image);

        this.roiAverage = Core.mean(this.image.submat(SignalSleeveROI)).val[0];

        if (this.roiAverage > this.THRESHOLD) return true;
        return false;
    }

    public int getParkingLocation() {
        return this.parkingLocation;
    }
}
