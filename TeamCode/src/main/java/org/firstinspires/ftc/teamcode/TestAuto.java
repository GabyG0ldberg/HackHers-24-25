package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous
public class TestAuto extends LinearOpMode {
    private HackHers_Lib everything;
    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;
    //DcMotor dw;
    DcMotor ls;
    //DcMotor om;
    OpenCvWebcam wc;
    Rev2mDistanceSensor ds1;
    Rev2mDistanceSensor ds2;

    @Override
    public void runOpMode() throws InterruptedException {
        fL = hardwareMap.get(DcMotor.class, "fl");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bl");
        bR = hardwareMap.get(DcMotor.class, "bR");
        //dw = hardwareMap.get(DcMotor.class, "dw");
        ls = hardwareMap.get(DcMotor.class, "ls");
        //om = hardwareMap.get(DcMotor.class, "om");
        ds1 = hardwareMap.get(Rev2mDistanceSensor.class, "ds1");
        ds2 = hardwareMap.get(Rev2mDistanceSensor.class, "ds2");
        wc = hardwareMap.get(OpenCvWebcam.class, "Webcam 1");

        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        super.waitForStart();

        everything = new HackHers_Lib(fL, fR, bL, bR, ls, ds1, ds2, wc);


        //THIS IS TEST CODE FOR ENCODER FUNCTIONS

        //everything.driveGoBackwardPosition(1480, .1);

        //while (bR.isBusy()) {
        //sleep(1);
        //}
        //everything.DriveBackwardToDist(50);

        everything.Stop();


        //everything.driveGoForwardPosition(1450, .1);
//        everything.driveTurnLeftPosition(100000, .2);
//        everything.driveTurnRightPosition(100000, .2);
    }
}