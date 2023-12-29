package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
@Disabled
public class HSVDetection extends OpenCvPipeline {
    /*
    GREEN  = Parking Left
    ORANGE    = Parking Middle
    PURPLE = Parking Right
     */

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // TOPLEFT anchor point for the bounding box
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(145, 168);

    // Width and height for the bounding box
    public static int REGION_WIDTH = 30;
    public static int REGION_HEIGHT = 50;

    // Lower and upper boundaries for colors
    private static final Scalar


// blue green and yellow

            lower_green_bounds  = new Scalar(42, 50, 50),
            upper_green_bounds  = new Scalar(76, 255, 255),
            lower_orange_bounds    = new Scalar(13, 50,50),
            upper_orange_bounds    = new Scalar(20, 255, 255),
            lower_purple_bounds = new Scalar(138, 50, 50),
            upper_purple_bounds = new Scalar(155, 255, 255);


    // Color definitions
    private final Scalar
            GREEN  = new Scalar(255, 255, 0),
            ORANGE    = new Scalar(0, 255, 255),
            PURPLE = new Scalar(255, 0, 255);

    // Percent and mat definitions
    private double grePercent, oraPercent, purPercent;
    private Mat greMat = new Mat(), oraMat = new Mat(), purMat = new Mat(), blurredMat = new Mat(), kernel = new Mat();

    // Anchor point definitions
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Running variable storing the parking position
    private volatile ParkingPosition position = ParkingPosition.LEFT;

    @Override
    public Mat processFrame(Mat input) {
        // Noise reduction
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        Imgproc.blur(input, blurredMat, new Size(5, 5));
        blurredMat = blurredMat.submat(new Rect(sleeve_pointA, sleeve_pointB));

        // Apply Morphology
        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(blurredMat, lower_green_bounds, upper_green_bounds, greMat);
        Core.inRange(blurredMat, lower_orange_bounds, upper_orange_bounds, oraMat);
        Core.inRange(blurredMat, lower_purple_bounds, upper_purple_bounds, purMat);

        // Gets color specific values
        grePercent = Core.countNonZero(greMat);
        oraPercent = Core.countNonZero(oraMat);
        purPercent = Core.countNonZero(purMat);

        // Calculates the highest amount of pixels being covered on each side
        double maxPercent = Math.max(grePercent, Math.max(oraPercent, purPercent));

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
        if (maxPercent == grePercent) {
            position = ParkingPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    GREEN,
                    2
            );
        } else if (maxPercent == oraPercent) {
            position = ParkingPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    ORANGE,
                    2
            );
        } else if (maxPercent == purPercent) {
            position = ParkingPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    PURPLE,
                    2
            );
        }

        // Memory cleanup
        blurredMat.release();
        greMat.release();
        oraMat.release();
        purMat.release();
        kernel.release();

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }
}