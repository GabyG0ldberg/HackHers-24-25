package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.auton.AprilTagAutonomousInitDetectionExample.FEET_PER_METER;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

import java.util.ArrayList;




// gaby Test except it strafes instead of turns
    @Autonomous
    public class MariaTest extends LinearOpMode {
        OpenCvWebcam camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;
        int LEFT = 12;
        int MIDDLE = 16;
        int RIGHT = 5;
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;
        double tagsize = 0.166;
        public static final double FEET_PER_METER = 3.28084;
        private HackHers_Lib everything;
        DcMotor fL;
        DcMotor fR;
        DcMotor bL;
        DcMotor bR;
        DcMotor ls;
        Servo cl;
        //Rev2mDistanceSensor ds1;
        //Rev2mDistanceSensor ds2;

        AprilTagDetection tagOfInterest = null;

        @Override
        public void runOpMode() throws InterruptedException {
            fL = hardwareMap.get(DcMotor.class, "fl");
            fR = hardwareMap.get(DcMotor.class, "fR");
            bL = hardwareMap.get(DcMotor.class, "bl");
            bR = hardwareMap.get(DcMotor.class, "bR");
            //t = telemetry.addData("count").get(Telemetry.class, "t");
            ls = hardwareMap.get(DcMotor.class, "ls");
            cl = hardwareMap.get(Servo.class, "cl");
            //ds1 = hardwareMap.get(Rev2mDistanceSensor.class, "ds1");
            //ds2 = hardwareMap.get(Rev2mDistanceSensor.class, "ds2");
            //cs = hardwareMap.get(ColorSensor.class, "cs");
            everything = new HackHers_Lib(fL, fR, bL, bR, ls, cl, camera);
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.getAll(WebcamName.class).get(0), cameraMonitorViewId);
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

            camera.setPipeline(aprilTagDetectionPipeline);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });


            telemetry.setMsTransmissionInterval(50);

            /*
             * The INIT-loop:
             * This REPLACES waitForStart!
             */
            while (!isStarted() && !isStopRequested()) {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                if (currentDetections.size() != 0) {
                    boolean tagFound = false;

                    for (AprilTagDetection tag : currentDetections) {
                        if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }

                    if (tagFound) {
                        telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                        tagToTelemetry(tagOfInterest);
                    } else {
                        telemetry.addLine("Don't see tag of interest :(");

                        if (tagOfInterest == null) {
                            telemetry.addLine("(The tag has never been seen)");
                        } else {
                            telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                            tagToTelemetry(tagOfInterest);
                        }
                    }

                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }

                }

                telemetry.update();
                sleep(20);
            }

            /*
             * The START command just came in: now work off the latest snapshot acquired
             * during the init loop.
             */

            super.waitForStart();

            /* Update the telemetry */
            if (tagOfInterest != null) {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            } else {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }

            /* Actually do something useful */
            if(tagOfInterest ==null||tagOfInterest.id ==LEFT)
            {
               /* everything.goBackward(.2);
                sleep(700);
                everything.Stop();
                everything.turnRight(.2);
                sleep(2200); */

                everything.strafeRight(.2);
                sleep(4125);
                everything.Stop();
                everything.goBackward(.2);
                sleep(3000);
                everything.Stop();
            } else if(tagOfInterest.id ==MIDDLE) {
                everything.goBackward(.2);
                sleep(3500);
                everything.Stop();
            } else {
                everything.strafeLeft(.2);
                sleep(4500);
                everything.Stop();
                everything.goBackward(.2);
                sleep(3000);
                everything.Stop();
            }


        }

        void tagToTelemetry(AprilTagDetection detection)
        {
            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
            telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
            telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
            telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
            telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
            telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
            telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        }
    }

