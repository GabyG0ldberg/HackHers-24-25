package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class BlueDuckAutonomous extends LinearOpMode {
    private HackHers_Lib everything;
    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;
    //Telemetry t;

    //DcMotor dw;
    DcMotor ls;
    //DcMotor om;
    Rev2mDistanceSensor ds1;
    Rev2mDistanceSensor ds2;
    //ColorSensor cs;
    OpenCvWebcam wc;


    @Override
    public void runOpMode() throws InterruptedException {
        fL = hardwareMap.get(DcMotor.class, "fl");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bl");
        bR = hardwareMap.get(DcMotor.class, "bR");
        //t = telemetry.addData("count").get(Telemetry.class, "t");
        //dw = hardwareMap.get(DcMotor.class, "dw");
        ls = hardwareMap.get(DcMotor.class, "ls");
        //om = hardwareMap.get(DcMotor.class, "om");
        ds1 = hardwareMap.get(Rev2mDistanceSensor.class,"ds1");
        ds2 = hardwareMap.get(Rev2mDistanceSensor.class,"ds2");
        //cs = hardwareMap.get(ColorSensor.class, "cs");
        int webcamID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        wc = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.getAll(WebcamName.class).get(0), webcamID);

        //wc = hardwareMap.get(OpenCvWebcam.class, "Webcam 1");



        super.waitForStart();
        everything = new HackHers_Lib(fL, fR, bL, bR, ls, ds1, ds2, wc);

        //THIS ABSOLUTELY FUNCTIONS AND MAKES A DUCK COME OFF BLUE SIDE
        //DO NOT EDIT

         everything.goForward(.2);

         sleep(1500);

         everything.Stop();

        everything.goBackward(.2);

        sleep(1500);

        everything.Stop();

        everything.turnRight(.2);

        sleep(1500);

        everything.Stop();

        everything.turnLeft(.2);

        sleep(1500);

        everything.Stop();

        everything.strafeRight(.2);

        sleep(1500);

        everything.Stop();

        everything.strafeLeft(.2);

        sleep(1500);

        everything.Stop();

        while (ds1.getDistance(DistanceUnit.CM) >= 10) {
            everything.goForward(.2);
            if (ds1.getDistance(DistanceUnit.CM) <= 10) {
                sleep(1500);
                everything.Stop();
            }
        }

        everything.goBackward(.2);

        sleep(1500);

        everything.Stop();

        //everything.goBackward(.2);


      //  sleep(750);

        //everything.DriveBackwardToDist(20);

        //everything.driveGoBackwardPosition(1680, .2);

        //everything.Stop();

       // everything.turnLeft(.2);

       // sleep(1700);

       // everything.Stop();

       //  everything.goForward(.2);

       //  sleep(1700);

        //everything.DriveForwardToDist(50, .2);

        // everything.turnRight(.2);

       // sleep(1400);

        // everything.Stop();

        //dw.setPower(-1);

       // sleep(3000);

        //dw.setPower(0);

       // everything.turnLeft(.2);

       // sleep(3200);

       // everything.Stop();

       // everything.goForward(.2);

       // sleep(1800);

        //everything.DriveForwardToDist(50, .2);

      //  everything.Stop();

//        everything.turnRight(.2);
//
//        sleep(1900);
//
//        everything.Stop();
//
//        everything.goForward(.3);
//
//        sleep(500);
//
//        everything.Stop();

    }


}
