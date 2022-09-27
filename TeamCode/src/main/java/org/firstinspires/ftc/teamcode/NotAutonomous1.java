package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class
git NotAutonomous1 extends OpMode {
    HackHers_Lib library;
    float [] omniValues = new float [4];
    int step = 1;
    ElapsedTime timer;
    public void delay(double delay) {
        double endTime = timer.milliseconds() + delay;
        while (timer.milliseconds() <= endTime) {
            //do nothing
        }


    }
    public void init() {
        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor br = hardwareMap.get(DcMotor.class, "br");
        DcMotor bl = hardwareMap.get(DcMotor.class, "bl");
        DcMotor dw = hardwareMap.get(DcMotor.class, "dw");
        DcMotor im = hardwareMap.get(DcMotor.class, "im");
        DcMotor om = hardwareMap.get(DcMotor.class, "om");
        Rev2mDistanceSensor ds1 = hardwareMap.get(Rev2mDistanceSensor.class, "ds1");
        Rev2mDistanceSensor ds2 = hardwareMap.get(Rev2mDistanceSensor.class, "ds2");

        HackHers_Lib library = new HackHers_Lib(fl, fr, bl, br, dw, im, om, ds1, ds2, telemetry);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void loop(){
        library.driveTurnLeftPosition(50, 0.7);
    }





}

