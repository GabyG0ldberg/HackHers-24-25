package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous
public class BlueParkAutonomous extends LinearOpMode {
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

        super.waitForStart();

        everything = new HackHers_Lib(fL, fR, bL, bR, telemetry);

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
