package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Range;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the skystone when lined up with
 * the sample regions over the first 3 stones.
 */
@TeleOp
public class MosaicDetectorExampleRED extends LinearOpMode
{

    OpenCvInternalCamera camera;
    MosaicDeterminationPipeline pipeline;

    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new MosaicDeterminationPipeline();
        camera.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public static class MosaicDeterminationPipeline extends OpenCvPipeline
    {
        static final Rect LeftROI = new Rect(
                new Point(30,290),
                new Point(79,335));
        static final Rect MiddleROI = new Rect(
                new Point(345,290),
                new Point(375,380));
        static final Rect RightROI = new Rect(
                new Point(450,290),
                new Point(79,335));

        static double PERCENT_COLOR_THRESHOLD = 0.4;

        Telemetry telemetry;
        Mat frame = new Mat();
        public enum Location {
            LEFT,
            RIGHT,
            NOT_FOUND
        }
        /*
         * An enum to define the skystone position
         */
        public enum SkystonePosition
        {
            LEFT,
            CENTER,
            RIGHT
        }



        @Override
        public void init(Mat firstFrame)
        {

        }
       // public MosaicDetectorExampleRED(Telemetry t) { telemetry = t; }
        @Override
        public Mat processFrame(Mat input) {


            Imgproc.cvtColor(input, frame, Imgproc.COLOR_RGB2HSV);
            Scalar lowerRed = new Scalar(0, 94, 87);         // lower color border for RED
            Scalar upperRed = new Scalar(0, 68, 100);
            Core.inRange(frame, lowerRed, upperRed, frame);

            Mat left = frame.submat(LeftROI);
            Mat right = frame.submat(RightROI);
            Mat middle = frame.submat(MiddleROI);
            double leftValue = Core.sumElems(left).val[0] / LeftROI.area() / 255;
            double rightValue = Core.sumElems(right).val[0] / RightROI.area() / 255;
            double middleValue = Core.sumElems(right).val[0] / MiddleROI.area() / 255;

            left.release();
            right.release();
            middle.release();

            telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
            telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
            telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
            telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
            telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
            telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");

            boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
            boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;
            boolean stoneMiddle = middleValue > PERCENT_COLOR_THRESHOLD;

            if (stoneLeft)

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

        }     */


}

