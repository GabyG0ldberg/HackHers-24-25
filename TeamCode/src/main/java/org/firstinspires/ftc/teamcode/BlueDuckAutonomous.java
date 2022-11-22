package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class BlueDuckAutonomous extends LinearOpMode {
    private HackHers_Lib everything;
    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;
    DcMotor dw;
    DcMotor im;
    DcMotor om;
    Rev2mDistanceSensor ds1;
    Rev2mDistanceSensor ds2;
    ColorSensor cs;
    //OpenCvWebcam wc;


    @Override
    public void runOpMode() throws InterruptedException {
        fL = hardwareMap.get(DcMotor.class, "fl");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bl");
        bR = hardwareMap.get(DcMotor.class, "bR");
        dw = hardwareMap.get(DcMotor.class, "dw");
        im = hardwareMap.get(DcMotor.class, "im");
        om = hardwareMap.get(DcMotor.class, "om");
        ds1 = hardwareMap.get(Rev2mDistanceSensor.class,"ds1");
        ds2 = hardwareMap.get(Rev2mDistanceSensor.class,"ds2");
        cs = hardwareMap.get(ColorSensor.class, "cs");
        //wc = hardwareMap.get(OpenCvWebcam.class, "Webcam 1");



        super.waitForStart();

        //everything = new HackHers_Lib(fL, fR, bL, bR, telemetry, wc);

        //THIS ABSOLUTELY FUNCTIONS AND MAKES A DUCK COME OFF BLUE SIDE
        //DO NOT EDIT

        everything.goBackward(.2);

        sleep(750);
        //everything.DriveBackwardToDist(20);

        //everything.driveGoBackwardPosition(1680, .2);

        everything.Stop();

        everything.turnLeft(.2);

        sleep(1700);

        everything.Stop();

        everything.goForward(.2);

        sleep(1700);

        //everything.DriveForwardToDist(50, .2);

        everything.turnRight(.2);

        sleep(1400);

        everything.Stop();

        dw.setPower(-1);

        sleep(3000);

        dw.setPower(0);

        everything.turnLeft(.2);

        sleep(3200);

        everything.Stop();

        everything.goForward(.2);

        sleep(1800);

        //everything.DriveForwardToDist(50, .2);

        everything.Stop();

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
