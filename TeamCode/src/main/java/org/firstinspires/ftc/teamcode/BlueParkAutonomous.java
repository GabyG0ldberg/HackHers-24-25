package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous
public class BlueParkAutonomous extends LinearOpMode {
    private HackHers_Lib everything;
    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;

    DcMotor ls;
    Servo cl;
    Rev2mDistanceSensor ds1;
    Rev2mDistanceSensor ds2;
    OpenCvWebcam wc;

    @Override
    public void runOpMode() throws InterruptedException {
        fL = hardwareMap.get(DcMotor.class, "fl");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bl");
        bR = hardwareMap.get(DcMotor.class, "bR");
        ls = hardwareMap.get(DcMotor.class, "ls");
        cl = hardwareMap.get(Servo.class, "cl");
        ds1 = hardwareMap.get(Rev2mDistanceSensor.class, "ds1");
        ds2 = hardwareMap.get(Rev2mDistanceSensor.class, "ds2");
        wc = hardwareMap.get(OpenCvWebcam.class, "Webcam 1");
        //ColorSensor sensor = hardwareMap.get(ColorSensor.class, "CS");




        super.waitForStart();

        everything = new HackHers_Lib(fL, fR, bL, bR, ls, cl, ds1, ds2, wc );

        everything.goBackward(.2);

        sleep(1350);

        everything.Stop();

        everything.turnRight(.2);

        // zev says hi

        sleep(2000);

        everything.Stop();

        everything.goForward(.8);

        sleep(1200);

        everything.Stop();


    }


}
