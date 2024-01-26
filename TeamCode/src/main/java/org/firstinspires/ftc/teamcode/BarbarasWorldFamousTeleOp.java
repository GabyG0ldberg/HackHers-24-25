package org.firstinspires.ftc.teamcode;


import static java.lang.Math.PI;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;



@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class BarbarasWorldFamousTeleOp extends OpMode {
    private HackHers_Lib everything;
    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;
    DcMotorEx ar;
    private OpenCvWebcam wc;

    CRServo cl; //this lie

    float  armHorizontal = 91;
    float armVertical = 346;
    float ninety = (float) (Math.PI)/2;

    float kff = 1;

    float armAngle = (0- armHorizontal )*(ninety/(armVertical - armHorizontal));
    double armAngleDegrees = armAngle*(180/Math.PI);
    double F = kff*Math.cos(armAngle);

    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;
    public static double NEW_F = 0.5;


    public void init() {
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bl");
        bR = hardwareMap.get(DcMotor.class, "bR");
        ar = hardwareMap.get(DcMotorEx.class, "ar");
        //cl = hardwareMap.get(Servo.class, "cl");  //this lie
        cl = hardwareMap.get(CRServo.class, "cl");  //this lie

        int webcamID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        wc = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.getAll(WebcamName.class).get(0), webcamID);

        everything = new HackHers_Lib(fL, fR, bL, bR, wc, ar, cl);//this lie
        ar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ar.setDirection(DcMotor.Direction.FORWARD);
        ar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfOrig = ar.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfNew = new PIDFCoefficients(pidfOrig.p, pidfOrig.i, pidfOrig.d, NEW_F);
        //ar.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        PIDFCoefficients pidfModified = ar.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Runtime (sec)", "%.01f", getRuntime());
        telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f",
                pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
        telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.04f",
                pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);
        telemetry.update();




    }



    @Override
    public void loop() {
//        float armAngle = (ar.getCurrentPosition()- armHorizontal )*(ninety/(armVertical - armHorizontal));
//        double armAngleDegrees = armAngle*(180/Math.PI);
//        double F = kff*Math.cos(armAngle);
        NEW_F = 4;


        everything.omniDrive(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
        int currentArmPosition = ar.getCurrentPosition();
        if (gamepad1.dpad_down) {
            cl.setDirection(DcMotorSimple.Direction.FORWARD);
            cl.setPower(1);
            telemetry.addLine("OPEN CLAW");
            telemetry.update();
        }
        if (gamepad1.dpad_up) {
            cl.setDirection(DcMotorSimple.Direction.FORWARD);
            cl.setPower(-1);
            telemetry.addLine("CLOSE CLAW");
            telemetry.update();
        }

        if (gamepad1.b) {//GOES DOWN / BACKWARDS BIG
            ar.setTargetPosition(0);
            ar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ar.setVelocity(1300);
        }


        if (gamepad1.y) { //STOPS
            ar.setVelocity(0);
        }

        if (gamepad1.a) { //GOES DOWN / BACKWARDS SMALL
            ar.setTargetPosition(ar.getCurrentPosition()+70);
            ar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ar.setVelocity(1300);
        }

        if (gamepad1.x) { //GOES UP / FORWARD BIG

            ar.setTargetPosition(-4300);
            ar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ar.setVelocity(1300);
            if(ar.getCurrentPosition()== ar.getTargetPosition()){
                ar.setVelocity(0);
            }
        }

        if (gamepad1.dpad_left) {//GOES DOWN / BACKWARDS BIG
            //ar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //ar.setDirection(DcMotor.Direction.FORWARD);
            ar.setTargetPosition(4300);
            ar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ar.setVelocity(1300);
            while (Math.abs(currentArmPosition) < Math.abs(ar.getTargetPosition()))
            {
                telemetry.addData("encoder-ARM", ar.getCurrentPosition());
                telemetry.update();
            }

        }


        if (gamepad1.right_bumper) {
            ar.setVelocity(1500);
        }

        if (gamepad1.left_bumper) {
            ar.setVelocity(-1500);
        }




        telemetry.addData("arm motor encoder", currentArmPosition);
        telemetry.addData("angle of arm", armAngleDegrees);

        telemetry.update();


    }
}
