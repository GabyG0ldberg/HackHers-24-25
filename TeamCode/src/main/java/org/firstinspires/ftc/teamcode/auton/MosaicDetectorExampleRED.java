package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
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
public class MosaicDetectorExampleRED extends LinearOpMode {

    OpenCvInternalCamera camera;
    MosaicDeterminationPipelineRED pipeline;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new MosaicDeterminationPipelineRED();
        camera.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addLine("Camera was opened");
                camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addLine("The camera cannot be opened");
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

             //Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }


    public static class MosaicDeterminationPipelineRED extends OpenCvPipeline {
        Mat frame = new Mat();

        Telemetry telemetry;

        public enum Location {
            LEFT,
            RIGHT,
            MIDDLE
        }
        private Location location = Location.RIGHT;
        static final Rect LeftROI = new Rect(
                new Point(60, 35),
                new Point(120, 75));
        static final Rect MiddleROI = new Rect(
                new Point(121, 80),
                new Point(121, 85));
        static final Rect RightROI = new Rect(
                new Point(140, 35),
                new Point(200, 75));
        static double PERCENT_COLOR_THRESHOLD = 0.4;

        //public volatile MosaicDetectorExampleRED(Telemetry t) { telemetry = t; }

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
            double middleValue = Core.sumElems(middle).val[0] / MiddleROI.area() / 255;

            left.release();
            right.release();
            middle.release();

//            telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
//            telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
//            telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
//            telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
//            telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
//            telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");

            boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
            boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;
            boolean stoneMiddle = middleValue > PERCENT_COLOR_THRESHOLD;

            if (stoneLeft) {
                location = Location.LEFT;
                //telemetry.addData("PROP Location", "LEFT");
            }
            else if (stoneRight) {
                location = Location.RIGHT;
                //telemetry.addData("PROP Location", "right");
            }
            else if (stoneMiddle) {
                location = Location.MIDDLE;
                //telemetry.addData("PROP Location", "MIDDLE");
            }
            else{
                //telemetry.addData("PROP Location", "NO PROP");
            }

            //telemetry.update();

            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_GRAY2RGB);


            Scalar colorSkystone = new Scalar(0, 255, 0);

            Imgproc.rectangle(frame, LeftROI, colorSkystone);
            Imgproc.rectangle(frame, RightROI, colorSkystone);
            Imgproc.rectangle(frame, MiddleROI, colorSkystone);


            return frame;

        }

        public Location getAnalysis()
        {
//
            return location;
        }
    }
}

