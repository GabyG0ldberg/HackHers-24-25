package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.io.BufferedInputStream;

// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
    
    @Autonomous(name="GabyTestJr")
    public class GabyTestJr extends LinearOpMode {
        private HackHers_Lib everything;
        DcMotor fL;
        DcMotor fR;
        DcMotor bL;
        DcMotor bR;

        @Override
        public void runOpMode() throws InterruptedException {
            fL = hardwareMap.get(DcMotor.class, "fl");
            fR = hardwareMap.get(DcMotor.class, "fR");
            bL = hardwareMap.get(DcMotor.class, "bl");
            bR = hardwareMap.get(DcMotor.class, "bR");

            // You will need to set this based on your robot's
            // gearing to get forward control input to result in
            // forward motion.
            fL.setDirection(DcMotor.Direction.FORWARD);
            fR.setDirection(DcMotor.Direction.FORWARD);
            bL.setDirection(DcMotor.Direction.FORWARD);
            bR.setDirection(DcMotor.Direction.FORWARD);


            // reset encoder counts kept by motors.
            fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set motors to run forward for 5000 encoder counts.
            fL.setTargetPosition(-200);
            fR.setTargetPosition(200);
            bL.setTargetPosition(200);
            bR.setTargetPosition(200);

            // set motors to run to target encoder position and stop with brakes on.
            fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("Mode", "waiting");
            telemetry.update();

            // wait for start button.

            waitForStart();

            telemetry.addData("Mode", "running");
            telemetry.update();

            // set both motors to 25% power. Movement will start. Sign of power is
            // ignored as sign of target encoder position controls direction when
            // running to position.

            fL.setPower(0); //motor 1
            fR.setPower(0); //motor 2
            bL.setPower(0.05); //motor 3
            bR.setPower(0.25); //motor 4

            // wait while opmode is active and left motor is busy running to position.

            while (opModeIsActive() && fL.isBusy())  //fL.getCurrentPosition() < fL.getTargetPosition())
            {
                telemetry.addData("encoder-fwd-left", fL.getCurrentPosition() + "  busy=" + fL.isBusy());
                telemetry.addData("encoder-fwd-right", fR.getCurrentPosition() + "  busy=" + fR.isBusy());
                telemetry.addData("encoder-back-left", bL.getCurrentPosition() + "  busy=" + bL.isBusy());
                telemetry.addData("encoder-back-right", bR.getCurrentPosition() + "  busy=" + bR.isBusy());
                telemetry.update();
                idle();
            }

            // set motor power to zero to turn off motors. The motors stop on their own but
            // power is still applied so we turn off the power.

            fL.setPower(0.0);
            fR.setPower(0.0);
            bL.setPower(0.0);
            bR.setPower(0.0);



            // wait 5 sec to you can observe the final encoder position.

            /*resetRuntime();

            while (opModeIsActive() && getRuntime() < 5)
            {
                telemetry.addData("encoder-fwd-left-end", fL.getCurrentPosition());
                telemetry.addData("encoder-fwd-right-end", fR.getCurrentPosition());
                telemetry.update();
                idle();
            }

            // From current position back up to starting point. In this example instead of
            // having the motor monitor the encoder we will monitor the encoder ourselves.

            fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            fL.setTargetPosition(0);
            fR.setTargetPosition(0);

            // Power sign matters again as we are running without encoder.
            fL.setPower(-0.05);
            fR.setPower(-0.05);

            while (opModeIsActive() && fL.getCurrentPosition() > fL.getTargetPosition())
            {
                telemetry.addData("encoder-back-left", fL.getCurrentPosition());
                telemetry.addData("encoder-back-right", fR.getCurrentPosition());
                telemetry.update();
                idle();
            }

            // set motor power to zero to stop motors.

            fL.setPower(0.0);
            fR.setPower(0.0);

            resetRuntime();

            while (opModeIsActive() && getRuntime() < 5)
            {
                telemetry.addData("encoder-back-left-end", fL.getCurrentPosition());
                telemetry.addData("encoder-back-right-end", fR.getCurrentPosition());
                telemetry.update();
                idle();
            }

             */
        }



    }



