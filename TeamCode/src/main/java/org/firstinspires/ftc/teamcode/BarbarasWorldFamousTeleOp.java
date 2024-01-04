package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class BarbarasWorldFamousTeleOp extends OpMode {
    private HackHers_Lib everything;
    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;
    DcMotor ar;
    private OpenCvWebcam wc;

    CRServo cl; //this lie

    //Servo cl;

  //  DcMotor ls;
    //Servo cl;
    //Rev2mDistanceSensor ds1;
    //Rev2mDistanceSensor ds2;
    //comment for the sake of existing

    public void init() {
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bl");
        bR = hardwareMap.get(DcMotor.class, "bR");
        ar = hardwareMap.get(DcMotor.class, "ar");
        //cl = hardwareMap.get(Servo.class, "cl");  //this lie
        cl = hardwareMap.get(CRServo.class, "cl");  //this lie

        int webcamID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        wc = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.getAll(WebcamName.class).get(0), webcamID);
      //  ls = hardwareMap.get(DcMotor.class, "ls");
        //cl = hardwareMap.get(Servo.class,"cl");
       // ds1 = hardwareMap.get(Rev2mDistanceSensor.class, "ds1");
       // ds2 = hardwareMap.get(Rev2mDistanceSensor.class, "ds2");
        everything = new HackHers_Lib(fL, fR, bL, bR, wc, ar, cl);//this lie

    }


    @Override
    public void loop() {
        everything.omniDrive(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

        if (gamepad1.dpad_down) {
            cl.setDirection(DcMotorSimple.Direction.FORWARD);
            cl.setPower(1);
            telemetry.addLine("DOWN DPAD");
            telemetry.update();
        }
        if (gamepad1.dpad_up) {
            cl.setDirection(DcMotorSimple.Direction.REVERSE);
            cl.setPower(1);
            telemetry.addLine("UP DPAD");
            telemetry.update();
        }
        if (gamepad1.dpad_right) {
            cl.setDirection(DcMotorSimple.Direction.REVERSE);
            cl.setPower(0);
            telemetry.addLine("RIGHT DPAD");
            telemetry.update();
        }
        if (gamepad1.dpad_left) {
            cl.setDirection(DcMotorSimple.Direction.FORWARD);
            cl.setPower(0);
            telemetry.addLine("LEFT DPAD");
            telemetry.update();
        }
//        if (gamepad1.y) {
//            everything.setMotorPower(ar, -0.2);
//        }
        if (gamepad1.a) {
            everything.armDown();
        }
        if (gamepad1.b) {
            everything.armStop();
        }

        if (gamepad1.right_bumper) { //claw closes a set amount
            everything.Open();
        }
        if (gamepad1.left_bumper) { //open
            everything.Close();
        }
        if (gamepad1.x) { //claw stops/at middle
            everything.armUp();
        }


//        everything.setMotorPower(fL,0);
//        everything.setMotorPower(fR,0);
//        everything.setMotorPower(bL,0);
//        everything.setMotorPower(bL,0);
        //everything.setMotorPower(ls, 0);
    }
}
