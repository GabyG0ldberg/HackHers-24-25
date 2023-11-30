package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.io.BufferedInputStream;

// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.

@Autonomous(name="GabyTestJr")
public class GabyTestJr extends LinearOpMode {
    private HackHers_Lib everything;
    DcMotor ls;
    Servo cl;



    @Override
    public void runOpMode() throws InterruptedException {
        ls = hardwareMap.get(DcMotor.class, "ls");
        ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ls.setDirection(DcMotor.Direction.REVERSE);
        ls.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // reset encoder counts kept by motors.
        ls.setTargetPosition(3500);  // set motors to run forward for 3500 encoder counts.
        ls.setMode(DcMotor.RunMode.RUN_TO_POSITION); // set motors to run to target encoder position and stop with brakes on.
        telemetry.addData("Mode", "waiting");
        telemetry.update();
        waitForStart();// wait for start button.
        telemetry.addData("Mode", "running");
        telemetry.update();
        ls.setPower(0.6);
        while (ls.getCurrentPosition() < ls.getTargetPosition())  //fL.getCurrentPosition() < fL.getTargetPosition() //opModeIsActive() && ls.isBusy()
        {
            telemetry.addData("encoder-linear-slides", ls.getCurrentPosition() + "  busy=" + ls.isBusy());
            telemetry.update();
            idle();
        }
        ls.setPower(0);


        //return null;
    }

}