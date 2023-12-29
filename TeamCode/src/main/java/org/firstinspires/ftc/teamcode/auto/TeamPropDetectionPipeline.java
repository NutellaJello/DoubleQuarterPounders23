package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class TeamPropDetectionPipeline extends OpenCvPipeline {
    /*
     * An enum to define the team prop position
     */
    public enum TeamPropPosition
    {
        LEFT,
        CENTER,
        RIGHT
    }

    boolean viewportPaused;
    private OpenCvWebcam webcam;
    /*
     * Some color constants
     */
    public final Scalar RED = new Scalar(255, 0, 0);
    public final Scalar BLUE = new Scalar(0, 0, 255);
    public final Scalar GREEN = new Scalar(0, 255, 0);

    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point LEFT_REGION_TOPLEFT_POINT = new Point(0,98);
    static final Point CENTER_REGION_TOPLEFT_POINT = new Point(140,98);
    static final Point RIGHT_REGION_TOPLEFT_POINT = new Point(300,98);
    static final int REGION_WIDTH = 20;
    static final int REGION_HEIGHT = 40;

    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */
    Point left_region_pointA = new Point(
            LEFT_REGION_TOPLEFT_POINT.x,
            LEFT_REGION_TOPLEFT_POINT.y);
    Point left_region1_pointB = new Point(
            LEFT_REGION_TOPLEFT_POINT.x + REGION_WIDTH,
            LEFT_REGION_TOPLEFT_POINT.y + REGION_HEIGHT);
    Point center_region_pointA = new Point(
            CENTER_REGION_TOPLEFT_POINT.x,
            CENTER_REGION_TOPLEFT_POINT.y);
    Point center_region_pointB = new Point(
            CENTER_REGION_TOPLEFT_POINT.x + REGION_WIDTH,
            CENTER_REGION_TOPLEFT_POINT.y + REGION_HEIGHT);
    Point right_region_pointA = new Point(
            RIGHT_REGION_TOPLEFT_POINT.x,
            RIGHT_REGION_TOPLEFT_POINT.y);
    Point right_region_pointB = new Point(
            RIGHT_REGION_TOPLEFT_POINT.x + REGION_WIDTH,
            RIGHT_REGION_TOPLEFT_POINT.y + REGION_HEIGHT);

    /*
     * Working variables
     */
    Mat leftMat, centerMat, rightMat;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avgLeft, avgCenter, avgRight;

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile TeamPropPosition position = TeamPropPosition.LEFT;

    private Telemetry telemetry;

    public TeamPropDetectionPipeline(OpenCvWebcam webcam, Telemetry telemetry) {
        this.webcam = webcam;
        this.telemetry = telemetry;

    }

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    @Override
    public void init(Mat input)
    {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        inputToCb(input);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        leftMat = Cb.submat(new Rect(left_region_pointA, left_region1_pointB));
        centerMat = Cb.submat(new Rect(center_region_pointA, center_region_pointB));
        rightMat = Cb.submat(new Rect(right_region_pointA, right_region_pointB));

    }

    @Override
    public Mat processFrame(Mat input)
    {
        /*
         * Overview of what we're doing:
         *
         * We first convert to YCrCb color space, from RGB color space.
         * Why do we do this? Well, in the RGB color space, chroma and
         * luma are intertwined. In YCrCb, chroma and luma are separated.
         * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
         * are Y, the luma channel (which essentially just a B&W image), the
         * Cr channel, which records the difference from red, and the Cb channel,
         * which records the difference from blue. Because chroma and luma are
         * not related in YCrCb, vision code written to look for certain values
         * in the Cr/Cb channels will not be severely affected by differing
         * light intensity, since that difference would most likely just be
         * reflected in the Y channel.
         *
         * After we've converted to YCrCb, we extract just the 2nd channel, the
         * Cb channel. We do this because stones are bright yellow and contrast
         * STRONGLY on the Cb channel against everything else, including SkyStones
         * (because SkyStones have a black label).
         *
         * We then take the average pixel value of 3 different regions on that Cb
         * channel, one positioned over each stone. The brightest of the 3 regions
         * is where we assume the SkyStone to be, since the normal stones show up
         * extremely darkly.
         *
         * We also draw rectangles on the screen showing where the sample regions
         * are, as well as drawing a solid rectangle over top the sample region
         * we believe is on top of the SkyStone.
         *
         * In order for this whole process to work correctly, each sample region
         * should be positioned in the center of each of the first 3 stones, and
         * be small enough such that only the stone is sampled, and not any of the
         * surroundings.
         */

        /*
         * Get the Cb channel of the input frame after conversion to YCrCb
         */
        inputToCb(input);

        /*
         * Draw a rectangle showing sample region 1 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                left_region_pointA, // First point which defines the rectangle
                left_region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                4); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                center_region_pointA, // First point which defines the rectangle
                center_region_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                4); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 3 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                right_region_pointA, // First point which defines the rectangle
                right_region_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                4); // Thickness of the rectangle lines

        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */



        avgLeft = (int) Core.mean(leftMat).val[0];
        avgCenter = (int) Core.mean(centerMat).val[0];
        avgRight = (int) Core.mean(rightMat).val[0];

        /*
         * Find the max of the 3 averages
         */
        int maxOneTwo = Math.max(avgLeft, avgCenter);
        int max = Math.max(maxOneTwo, avgRight);

        /*
         * Now that we found the max, we actually need to go and
         * figure out which sample region that value was from
         */
        if(max == avgLeft) // Was it from region 1?
        {
            position = TeamPropPosition.LEFT; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    left_region_pointA, // First point which defines the rectangle
                    left_region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(max == avgCenter) // Was it from region 2?
        {
            position = TeamPropPosition.CENTER; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    center_region_pointA, // First point which defines the rectangle
                    center_region_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(max == avgRight) // Was it from region 3?
        {
            position = TeamPropPosition.RIGHT; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    right_region_pointA, // First point which defines the rectangle
                    right_region_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }

        telemetry.addData("[Pattern]", position);
        telemetry.addData("left", avgLeft);
        telemetry.addData("center", avgCenter);
        telemetry.addData("right", avgRight);
        telemetry.update();

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public TeamPropPosition getPosition()
    {
        return position;
    }


    public void onViewportTapped() {
        this.viewportPaused = !this.viewportPaused;
        if (this.viewportPaused) {
            this.webcam.pauseViewport();
        } else {
            this.webcam.resumeViewport();
        }
    }
}