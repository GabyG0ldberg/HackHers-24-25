package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.io.BufferedInputStream;

// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.

@Autonomous(name="GabyTestJr")
public class GabyTestJr extends LinearOpMode {
    private HackHers_Lib everything;
    DcMotorEx fL;
    DcMotorEx fR;
    DcMotorEx bL;
    DcMotorEx bR;

//ls.setDirection(DcMotor.Direction.REVERSE);

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx fL = hardwareMap.get(DcMotorEx.class, "fL");
        DcMotorEx fR = hardwareMap.get(DcMotorEx.class, "fR");
        DcMotorEx bL = hardwareMap.get(DcMotorEx.class, "bl");
        DcMotorEx bR = hardwareMap.get(DcMotorEx.class, "bR");


        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fL.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);


        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // reset encoder counts kept by motors.
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // reset encoder counts kept by motors.
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // reset encoder counts kept by motors.
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // reset encoder counts kept by motors.

        fL.setTargetPosition(3500);  // set motors to run forward for 3500 encoder counts.
        fR.setTargetPosition(3500);  // set motors to run forward for 3500 encoder counts.
        bL.setTargetPosition(3500);  // set motors to run forward for 3500 encoder counts.
        bR.setTargetPosition(3500);  // set motors to run forward for 3500 encoder counts.

        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // set motors to run to target encoder position and stop with brakes on.
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // set motors to run to target encoder position and stop with brakes on
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // set motors to run to target encoder position and stop with brakes on.
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // set motors to run to target encoder position and stop with brakes on


        telemetry.addData("Mode", "waiting");
        telemetry.update();
        waitForStart();// wait for start button.
        telemetry.addData("Mode", "running");
        telemetry.update();
        fL.setVelocity(1000);
        fR.setVelocity(1000);
        bL.setVelocity(1000);
        bR.setVelocity(1000);
        //fR.setPower(0.05);
        while (Math.abs(fL.getCurrentPosition()) < Math.abs(fL.getTargetPosition()) && Math.abs(fR.getCurrentPosition()) < Math.abs(fR.getTargetPosition()))  //fL.getCurrentPosition() < fL.getTargetPosition() //opModeIsActive() && ls.isBusy()
        {
            telemetry.addData("encoder-linear-slides", fL.getCurrentPosition() + "  busy=" + fL.isBusy());
            telemetry.addData("encoder-linear-slides", fR.getCurrentPosition() + "  busy=" + fR.isBusy());
            telemetry.addData("encoder-linear-slides", bL.getCurrentPosition() + "  busy=" + bL.isBusy());
            telemetry.addData("encoder-linear-slides", bR.getCurrentPosition() + "  busy=" + bR.isBusy());
            telemetry.update();
            idle();
        }
        fL.setVelocity(0);
        fR.setVelocity(0);
        bL.setVelocity(0);
        bR.setVelocity(0);


        //return null;
    }

}