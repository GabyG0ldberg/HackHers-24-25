
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.HackHers_Lib;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public abstract class TestTeleOp extends OpMode {
    private HackHers_Lib everything;
    DcMotor fL = hardwareMap.get(DcMotor.class, "fl");
    DcMotor fR = hardwareMap.get(DcMotor.class, "fR");
    DcMotor bL = hardwareMap.get(DcMotor.class, "bl");
    DcMotor bR = hardwareMap.get(DcMotor.class, "bR");
    DcMotor dw = hardwareMap.get(DcMotor.class, "dw");
    DcMotor im = hardwareMap.get(DcMotor.class, "im");
    DcMotor om = hardwareMap.get(DcMotor.class, "om");
    Rev2mDistanceSensor ds1 = hardwareMap.get(Rev2mDistanceSensor .class, "ds0");
    Rev2mDistanceSensor ds2 = hardwareMap.get(Rev2mDistanceSensor.class, "ds1");

    public void init() {

        everything = new HackHers_Lib(fL, fR, bL, bR, telemetry);
    }

    @Override
    public void loop() {
        everything.omniDrive(gamepad1.right_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
}