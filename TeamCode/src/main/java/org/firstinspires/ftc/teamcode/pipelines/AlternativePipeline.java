package org.firstinspires.ftc.teamcode.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class AlternativePipeline extends ParkingLocationPipeline {
    private int parkingLocation = 0;

    //Outputs
    private final Mat cvExtractchannel0Output = new Mat();
    private final Mat cvExtractchannel2Output = new Mat();
    private final Mat cvThreshold0Output = new Mat();
    private final Mat cvThreshold2Output = new Mat();

    public Rect SignalSleeveROI = new Rect(
            new Point(1000, 100),
            new Point(1100, 400)
    );

    @Override
    public Mat processFrame(Mat input) {
        cvExtractchannel(input, 0, cvExtractchannel0Output);
        cvExtractchannel(input, 2, cvExtractchannel2Output);

        cvThreshold(cvExtractchannel0Output, 59.0, 255, Imgproc.THRESH_BINARY_INV, cvThreshold0Output);
        cvThreshold(cvExtractchannel2Output, 91.0, 255, Imgproc.THRESH_BINARY_INV, cvThreshold2Output);

        double channel0Average = cvMean(cvExtractchannel0Output.submat(SignalSleeveROI));
        double channel2Average = cvMean(cvExtractchannel2Output.submat(SignalSleeveROI));

        if (channel0Average < 100) {
            parkingLocation = 3;
        } else if (channel2Average < 100) {
            parkingLocation = 1;
        } else {
            parkingLocation = 2;
        }

        Imgproc.rectangle(cvThreshold0Output, this.SignalSleeveROI, new Scalar(0, 255, 0), 4);

        return cvThreshold0Output;
    }

    /**
     * Extracts given channel from an image.
     * @param src the image to extract.
     * @param channel zero indexed channel number to extract.
     * @param dst output image.
     */
    private void cvExtractchannel(Mat src, int channel, Mat dst) {
        Core.extractChannel(src, dst, channel);
    }

    /**
     * Apply a fixed-level threshold to each array element in an image.
     * @param src Image to threshold.
     * @param threshold threshold value.
     * @param maxVal Maximum value for THRES_BINARY and THRES_BINARY_INV
     * @param type Type of threshold to appy.
     * @param dst output Image.
     */
    private void cvThreshold(Mat src, double threshold, double maxVal, int type,
                             Mat dst) {
        Imgproc.threshold(src, dst, threshold, maxVal, type);
    }

    private double cvMean(Mat src) {
        return Core.mean(src).val[0];
    }

    @Override
    public int getParkingLocation() {
        return parkingLocation;
    }
}
