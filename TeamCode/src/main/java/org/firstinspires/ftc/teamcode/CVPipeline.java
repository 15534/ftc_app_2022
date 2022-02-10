package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class CVPipeline extends OpenCvPipeline {
    public static final Point REGION1_A = new Point(205, 40);
    public static final Point REGION1_B = new Point(215, 50);

    public static final Point REGION2_A = new Point(220, 100);
    public static final Point REGION2_B = new Point(230, 110);

    public static final Point REGION3_A = new Point(245, 180);
    public static final Point REGION3_B = new Point(255, 190);


    Mat region1_Cb, region2_Cb, region3_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    private volatile int avg1, avg2, avg3;
    private volatile int position = 0;  // 0 - left, 1 - middle, 2 - right

    void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    @Override
    public void init(Mat firstFrame) {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        inputToCb(firstFrame);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        region1_Cb = Cb.submat(new Rect(REGION1_A, REGION1_B));
        region2_Cb = Cb.submat(new Rect(REGION2_A, REGION2_B));
        region3_Cb = Cb.submat(new Rect(REGION3_A, REGION3_B));
    }

    @Override
    public Mat processFrame(Mat input) {
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
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */
        avg1 = (int) Core.mean(region1_Cb).val[0];
        avg2 = (int) Core.mean(region2_Cb).val[0];
        avg3 = (int) Core.mean(region3_Cb).val[0];

        final Scalar RED = new Scalar(255, 0, 0);
        final Scalar BLUE = new Scalar(0, 0, 255);
        final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * Draw a rectangle showing sample region 1 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                REGION1_A, // First point which defines the rectangle
                REGION1_B, // Second point which defines the rectangle
                RED, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                REGION2_A, // First point which defines the rectangle
                REGION2_B, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                REGION3_A, // First point which defines the rectangle
                REGION3_B, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

//
//        if (avg1 < REG1_CUTOFF) {
//            stack = 4;
////                Imgproc.rectangle(
////                        input, // Buffer to draw on
////                        REGION1_A, // First point which defines the rectangle
////                        REGION1_B, // Second point which defines the rectangle
////                        GREEN, // The color the rectangle is drawn in
////                        -1);
//        } else if (avg2 < REG2_CUTOFF) {
//            stack = 1;
////                Imgproc.rectangle(
////                        input, // Buffer to draw on
////                        REGION2_A, // First point which defines the rectangle
////                        REGION2_B, // Second point which defines the rectangle
////                        GREEN, // The color the rectangle is drawn in
////                        -1);
//        } else {
//            stack = 0;
//        }

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }

    public int getPosition()
    {
        return position;
    }

    public int getAvg1() { return avg1; }
    public int getAvg2() { return avg2; }
    public int getAvg3() { return avg3; }
}