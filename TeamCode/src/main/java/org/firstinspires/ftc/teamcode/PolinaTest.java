package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.auton.PropDetectorThePipeLine;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;


import java.util.ArrayList;


@Autonomous
public class PolinaTest extends LinearOpMode {
    int width = 320;
    int height = 240;
    // store as variable here so we can access the location
    OpenCvCamera camera;
    PropDetectorThePipeLine pipeline;
    private HackHers_Lib everything;
    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;
    DcMotor ls;
    Servo cl;
   // Rev2mDistanceSensor ds1;
   // Rev2mDistanceSensor ds2;
    @Override
    public void runOpMode() throws InterruptedException {
        fL = hardwareMap.get(DcMotor.class, "fl");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bl");
        bR = hardwareMap.get(DcMotor.class, "bR");
        ls = hardwareMap.get(DcMotor.class, "ls");
        cl = hardwareMap.get(Servo.class, "cl");
        everything = new HackHers_Lib(fL, fR, bL, bR, ls, cl, (OpenCvWebcam) camera);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);


        PropDetectorThePipeLine detector = new PropDetectorThePipeLine(width);
        pipeline = new PropDetectorThePipeLine(640);

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.getAll(WebcamName.class).get(0), cameraMonitorViewId);
        camera.setPipeline(detector);


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
            PropDetectorThePipeLine.PropLocation location = detector.getLocation();
            if (location != PropDetectorThePipeLine.PropLocation.NONE) {
                telemetry.addLine("SAW SOMETHING MAYBE?!");
                telemetry.update();
                break;
            } else {
                telemetry.addLine("DIDNT SEE ANYTHING LOSER");
                telemetry.update();
            }
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        super.waitForStart();



        /* Actually do something useful */
        PropDetectorThePipeLine.PropLocation location = detector.getLocation();
        if (location != PropDetectorThePipeLine.PropLocation.NONE) {
            telemetry.addLine("SAW SOMETHING MAYBE?!");
            telemetry.update();
            everything.strafeRight(.3);
            sleep(2000);
            everything.Stop();
        } else {
            everything.strafeLeft(.3);
            sleep(2000);
            everything.Stop();
            telemetry.addLine("DIDNT SEE ANYTHING LOSER");
            telemetry.update();

        }


    }
}