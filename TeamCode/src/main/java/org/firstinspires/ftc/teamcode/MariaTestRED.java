package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.MosaicDetectorExample;
import org.firstinspires.ftc.teamcode.auton.MosaicDetectorExampleRED;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

// gaby Test except it strafes instead of turns
    @Autonomous
    public class MariaTestRED extends LinearOpMode {
    OpenCvWebcam camera;
    MosaicDetectorExampleRED.MosaicDeterminationPipeline pipeline;
    MosaicDetectorExampleRED.MosaicDeterminationPipeline.SkystonePosition snapshotAnalysis = MosaicDetectorExampleRED.MosaicDeterminationPipeline.SkystonePosition.LEFT;

    private HackHers_Lib everything;
    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;
    DcMotor ls;
    Servo cl;

    @Override
    public void runOpMode() throws InterruptedException {
        fL = hardwareMap.get(DcMotor.class, "fl");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bl");
        bR = hardwareMap.get(DcMotor.class, "bR");
        ls = hardwareMap.get(DcMotor.class, "ls");
        cl = hardwareMap.get(Servo.class, "cl");
        everything = new HackHers_Lib(fL, fR, bL, bR, ls, cl, camera);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.getAll(WebcamName.class).get(0), cameraMonitorViewId);
        pipeline = new MosaicDetectorExampleRED.MosaicDeterminationPipeline();
        camera.setPipeline(pipeline);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        //snapshotAnalysis = pipeline.getAnalysis();

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        telemetry.update();
        sleep(20);



        super.waitForStart();
        snapshotAnalysis = pipeline.getAnalysis();

        /*
         * Show that snapshot on the telemetry
         */
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        switch (snapshotAnalysis){
            case LEFT:
            {
                everything.goBackward(.3);
                sleep(2000);
                everything.Stop();
            }
            case RIGHT:
            {
                everything.goForward(.3);
                sleep(2000);
                everything.Stop();
            }
            case CENTER:
            {
                everything.strafeLeft(.3);
                sleep(2000);
                everything.Stop();
            }
        }

        /* Update the telemetry */



    }


    }

