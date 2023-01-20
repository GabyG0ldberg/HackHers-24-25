package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.auton.AprilTagAutonomousInitDetectionExample.FEET_PER_METER;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous
public class RedLeft extends LinearOpMode {
    private HackHers_Lib everything;
    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;
    //Telemetry t;
    DcMotor ls;
    Servo cl;
    int LEFT = 12;
    int MIDDLE = 16;
    int RIGHT = 5;
    //Rev2mDistanceSensor ds1;
    //Rev2mDistanceSensor ds2;
    //ColorSensor cs;
    OpenCvWebcam camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    AprilTagDetection tagOfInterest = null;
    // UNITS ARE METERS
    double tagsize = 0.166;

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

    @Override
    public void runOpMode() throws InterruptedException {
        fL = hardwareMap.get(DcMotor.class, "fl");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bl");
        bR = hardwareMap.get(DcMotor.class, "bR");
        //t = telemetry.addData("count").get(Telemetry.class, "t");
        ls = hardwareMap.get(DcMotor.class, "ls");
        cl = hardwareMap.get(Servo.class, "cl");
       // ds1 = hardwareMap.get(Rev2mDistanceSensor.class, "ds1");
        //ds2 = hardwareMap.get(Rev2mDistanceSensor.class, "ds2");
        //cs = hardwareMap.get(ColorSensor.class, "cs");
        everything = new HackHers_Lib(fL, fR, bL, bR, ls, cl, camera);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.getAll(WebcamName.class).get(0), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);


        telemetry.setMsTransmissionInterval(50);

        super.waitForStart();


        //THIS ABSOLUTELY FUNCTIONS AND MAKES A DUCK COME OFF BLUE SIDE
        //DO NOT EDIT

        everything.strafeRight(.2);

        sleep(2500);

        everything.Stop();

        everything.goForward(.2); /*to the cone */

        sleep(3000);

        everything.Stop();

        everything.turnRight(.2);

        sleep(1250);  /* 45 degree angle to face the cone*/

        everything.Stop();

        sleep(3000); //place cone

        everything.turnRight(.2); /* 135 degree turn to face the wall*/

        sleep(3200);

        everything.Stop();

        everything.goForward(.2); /*go forward to the wall*/

        sleep(2700);

        everything.Stop();

        everything.strafeRight(.2); /*to go in front of the camera*/

        sleep(4500);

        everything.Stop();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (opModeIsActive() && !isStopRequested()) {
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
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            everything.strafeLeft(.2);
            sleep(1500);
            everything.Stop();
            everything.goForward(.2);
            sleep(1500);
            everything.Stop();
        } else if (tagOfInterest.id == MIDDLE) {
            everything.goForward(.2);
            sleep(1500);
            everything.Stop();
        } else {
            everything.strafeRight(.2);
            sleep(1500);
            everything.Stop();
            everything.goForward(.2);
            sleep(1500);
            everything.Stop();
        }

        /*read cone*/
        /*park*/

    }


}

