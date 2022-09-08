package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class CaritaTeleOp extends OpMode {
        private HackHers_Lib everything;
        DcMotor fL;
        DcMotor fR;

        public void init() {
            fL = hardwareMap.get(DcMotor.class, "fl");
            fR = hardwareMap.get(DcMotor.class, "fR");
            everything = new HackHers_Lib(fL, fR, telemetry);


        }

        @Override
        public void loop() {
            everything.omniDrive(gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.right_stick_y);
            
        }
}