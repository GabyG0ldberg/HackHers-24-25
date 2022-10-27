package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class RedDuckAutonomous extends LinearOpMode {
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
    OpenCvWebcam wc;

    @Override
    public void runOpMode() throws InterruptedException {
        fL = hardwareMap.get(DcMotor.class, "fl");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bl");
        bR = hardwareMap.get(DcMotor.class, "bR");
        dw = hardwareMap.get(DcMotor.class, "dw");
        im = hardwareMap.get(DcMotor.class, "im");
        om = hardwareMap.get(DcMotor.class, "om");
        ds1 = hardwareMap.get(Rev2mDistanceSensor.class, "ds1");
        ds2 = hardwareMap.get(Rev2mDistanceSensor.class, "ds2");
        wc = hardwareMap.get(OpenCvWebcam.class, "Webcam 1");

        super.waitForStart();

        everything = new HackHers_Lib(fL, fR, bL, bR, telemetry, wc);



/**
        everything.turnRight(.2);
        // zev says hi
        sleep(2500);

        everything.Stop();

        everything.goForward(.2);

         sleep(1800);
         everything.Stop();

         dw.setPower(1);
         sleep(3000);
         dw.setPower(0);

        everything.turnRight(.2);
//
        sleep(2000);
        everything.Stop();
//
        everything.goForward(.2);
//
        sleep(2000);
//
        everything.Stop();

        everything.goBackward(.5);




//        everything.turnLeft(.2);
//
//        sleep(1300);
//        everything.Stop();
//
//        dw.setPower(1);
//
//        sleep(3000);
//
//        dw.setPower(0);
//
//        everything.goBackward(.2);
//
//        sleep(2000);
//
//        everything.Stop();
//
//        everything.turnRight(.2);
//
//        sleep(1400);
//        everything.Stop();
//
//        everything.goForward(.2);
//
//        sleep(2400);
//
//        everything.Stop();
//
//        everything.turnLeft(.2);
//        sleep(800);
//
//        everything.Stop();
*/

        everything.goBackward(.5);
        sleep(5000);

        everything.Stop();

        everything.turnLeft(2.0);
        sleep(4500);
        everything.Stop();







//        if (ts.isPressed() == true) {
//            fL.setPower(0);
//            fR.setPower(0);
//            bL.setPower(0);
//            bR.setPower(0);
//        }

    }

}
