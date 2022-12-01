package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class RedParkAutonomous extends LinearOpMode {
    private HackHers_Lib everything;
    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;
    DcMotor ls;
    CRServo cl;
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
        cl = hardwareMap.get(CRServo.class, "cl");
        ds1 = hardwareMap.get(Rev2mDistanceSensor.class, "ds1");
        ds2 = hardwareMap.get(Rev2mDistanceSensor.class, "ds2");
        wc = hardwareMap.get(OpenCvWebcam.class, "Webcam 1");

        super.waitForStart();

        everything = new HackHers_Lib(fL, fR, bL, bR, ls, cl, ds1, ds2, wc);

        //THIS TRIES TO GET IT TO PARK
        //DO NOT EDIT

        everything.goBackward(.2);

        sleep(2000);
        everything.Stop();

        everything.turnLeft(.2);

        // zev says hi

        sleep(2000);
          everything.Stop();

        everything.goForward(.8);

        sleep(1600);
        everything.Stop();

    }


}
