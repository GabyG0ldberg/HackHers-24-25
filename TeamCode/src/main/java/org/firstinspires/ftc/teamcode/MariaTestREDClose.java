package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.auton.MosaicDetectorExample.MosaicDeterminationPipeline.SkystonePosition.LEFT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;



import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.auton.MosaicDetectorExample;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous
public class MariaTestREDClose extends LinearOpMode {
    OpenCvWebcam camera;
    MosaicDetectorExample.MosaicDeterminationPipeline pipeline;
    MosaicDetectorExample.MosaicDeterminationPipeline.SkystonePosition snapshotAnalysis = LEFT;
    private HackHers_Lib everything;
    DcMotorEx fL;
    DcMotorEx fR;
    DcMotorEx bL;
    DcMotorEx bR;
    DcMotorEx ar;

    CRServo cl; //this lie

    public IMU imu;
    float targetAngle;
    float globalAngle;
    Orientation lastAngles = new Orientation();
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(IMU.class, "imu");

        fL = hardwareMap.get(DcMotorEx.class, "fL");
        fR = hardwareMap.get(DcMotorEx.class, "fR");
        bL = hardwareMap.get(DcMotorEx.class, "bl");
        bR = hardwareMap.get(DcMotorEx.class, "bR");
        ar = hardwareMap.get(DcMotorEx.class, "ar");
        cl = hardwareMap.get(CRServo.class, "cl");  //this lie

        everything = new HackHers_Lib(fL, fR, bL, bR, camera, ar, cl);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fL.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.getAll(WebcamName.class).get(0), cameraMonitorViewId);
        pipeline = new MosaicDetectorExample.MosaicDeterminationPipeline();
        camera.setPipeline(pipeline);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Realtime analysis!", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        telemetry.update();
        sleep(20);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        super.waitForStart();
        snapshotAnalysis = pipeline.getAnalysis();
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);

        // Retrieve Rotational Angles
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        telemetry.update();
        telemetry.addLine("start analysis");
        telemetry.update();

        switch (snapshotAnalysis) {
            case LEFT: {
                telemetry.addLine("left");
                telemetry.update();
                goBackward(1000);
                targetAngle = 90;
                turnRight();
                imu.resetYaw();
                //everything.strafeRight(.5);
                //drop
//                targetAngle = 20;
//                turnRight();
//                goForward(1800);
//                //drop
//                everything.armUp();
//                everything.Open();
//                everything.Close();
                break;

            }
            case RIGHT: {
                telemetry.addLine("right");
                telemetry.update();
                imu.resetYaw();
                goBackward(1000);
                everything.strafeLeft(.2);
                sleep(300);
                everything.Stop();
                targetAngle = 90;
                turnRight();
                imu.resetYaw();
                //drop
//                targetAngle = 180;
//                turnRight();
//                goForward(1800);
                //drop
                break;
            }
            case CENTER: {
                telemetry.addLine("center after start");
                telemetry.update();
                imu.resetYaw();
                goBackward(2500);
//                targetAngle = 180;
//                turnLeft();
//                imu.resetYaw();
//                //drop
//                targetAngle = 90;
//                turnRight();
//                imu.resetYaw();
//                goForward(1800  );
                //drop
                break;
            }
        }

    }

    public void goForward(int targetPosition) {
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // reset encoder counts kept by motors.
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // reset encoder counts kept by motors.
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // reset encoder counts kept by motors.
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // reset encoder counts kept by motors.
        fL.setTargetPosition(targetPosition);  // set motors to run forward for 3500 encoder counts.
        fR.setTargetPosition(targetPosition);  // set motors to run forward for 3500 encoder counts.
        bL.setTargetPosition(targetPosition);  // set motors to run forward for 3500 encoder counts.
        bR.setTargetPosition(targetPosition);  // set motors to run forward for 3500 encoder counts.
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // set motors to run to target encoder position and stop with brakes on.
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // set motors to run to target encoder position and stop with brakes on
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // set motors to run to target encoder position and stop with brakes on.
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // set motors to run to target encoder position and stop with brakes on
        telemetry.addData("Mode", "running");
        telemetry.update();
        fL.setVelocity(500);
        fR.setVelocity(500);
        bL.setVelocity(500);
        bR.setVelocity(500);
        //fR.setPower(0.05);
        while (Math.abs(fL.getCurrentPosition()) < Math.abs(fL.getTargetPosition()) && Math.abs(fR.getCurrentPosition()) < Math.abs(fR.getTargetPosition()))  //fL.getCurrentPosition() < fL.getTargetPosition() //opModeIsActive() && ls.isBusy()
        {
            telemetry.addData("encoder-FORWARD", fL.getCurrentPosition() + "  busy=" + fL.isBusy());
            telemetry.addData("encoder-FORWARD", fR.getCurrentPosition() + "  busy=" + fR.isBusy());
            telemetry.addData("encoder-FORWARD", bL.getCurrentPosition() + "  busy=" + bL.isBusy());
            telemetry.addData("encoder-FORWARD", bR.getCurrentPosition() + "  busy=" + bR.isBusy());
            telemetry.update();
            idle();
        }
        fL.setVelocity(0);
        fR.setVelocity(0);
        bL.setVelocity(0);
        bR.setVelocity(0);
    }
    public void goBackward(int targetPosition) {
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // reset encoder counts kept by motors.
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // reset encoder counts kept by motors.
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // reset encoder counts kept by motors.
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // reset encoder counts kept by motors.
        fL.setTargetPosition(targetPosition);  // set motors to run forward for 3500 encoder counts.
        fR.setTargetPosition(targetPosition);  // set motors to run forward for 3500 encoder counts.
        bL.setTargetPosition(targetPosition);  // set motors to run forward for 3500 encoder counts.
        bR.setTargetPosition(targetPosition);  // set motors to run forward for 3500 encoder counts.
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // set motors to run to target encoder position and stop with brakes on.
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // set motors to run to target encoder position and stop with brakes on
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // set motors to run to target encoder position and stop with brakes on.
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // set motors to run to target encoder position and stop with brakes on
        telemetry.addData("Mode", "running");
        telemetry.update();
        fL.setVelocity(-500);
        fR.setVelocity(-500);
        bL.setVelocity(-500);
        bR.setVelocity(-500);
        //fR.setPower(0.05);
        while (Math.abs(fL.getCurrentPosition()) < Math.abs(fL.getTargetPosition()) && Math.abs(fR.getCurrentPosition()) < Math.abs(fR.getTargetPosition()))  //fL.getCurrentPosition() < fL.getTargetPosition() //opModeIsActive() && ls.isBusy()
        {
            telemetry.addData("encoder-BACKWARD", fL.getCurrentPosition() + "  busy=" + fL.isBusy());
            telemetry.addData("encoder-BACKWARD", fR.getCurrentPosition() + "  busy=" + fR.isBusy());
            telemetry.addData("encoder-BACKWARD", bL.getCurrentPosition() + "  busy=" + bL.isBusy());
            telemetry.addData("encoder-BACKWARD", bR.getCurrentPosition() + "  busy=" + bR.isBusy());
            telemetry.update();
            idle();
        }
        fL.setVelocity(0);
        fR.setVelocity(0);
        bL.setVelocity(0);
        bR.setVelocity(0);
    }
    public void turnLeft() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        everything.turnLeft(.2);
        while (Math.abs(Math.abs(orientation.getYaw(AngleUnit.DEGREES)) - targetAngle) > 8) {
            orientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
        everything.Stop();
    }
    public void turnRight(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        everything.turnRight(.2);
        while (Math.abs(Math.abs(orientation.getYaw(AngleUnit.DEGREES)) - targetAngle) > 8) {
            orientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
        everything.Stop();
    }
}


