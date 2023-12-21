package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.auton.MosaicDetectorExample.MosaicDeterminationPipeline.SkystonePosition.LEFT;
import static org.firstinspires.ftc.teamcode.auton.MosaicDetectorExample.MosaicDeterminationPipeline.SkystonePosition.RIGHT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


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


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous
public class IMUTester extends LinearOpMode {
    OpenCvWebcam camera;
    MosaicDetectorExample.MosaicDeterminationPipeline pipeline;
    MosaicDetectorExample.MosaicDeterminationPipeline.SkystonePosition snapshotAnalysis = LEFT;
    private HackHers_Lib everything;
    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;
    DcMotor ar;
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

        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bl");
        bR = hardwareMap.get(DcMotor.class, "bR");
        ar = hardwareMap.get(DcMotor.class, "ar");
        everything = new HackHers_Lib(fL, fR, bL, bR, camera, ar);
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
        double actualAngle = orientation.getYaw(AngleUnit.DEGREES);
        telemetry.addLine("start analysis");
        telemetry.update();

        switch (snapshotAnalysis) {
            case LEFT: {
                telemetry.addLine("left");
                telemetry.update();
                imu.resetYaw();
                targetAngle = 90;
                everything.turnLeft(.2);
                while (Math.abs(Math.abs(orientation.getYaw(AngleUnit.DEGREES)) - targetAngle) > 8) {
                    orientation = imu.getRobotYawPitchRollAngles();
                    telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                    telemetry.update();
                }
            }
            case RIGHT:{
                //System.out.println("turning right");
                telemetry.addLine("right");
                telemetry.update();

                imu.resetYaw();
                targetAngle = 90;
                everything.turnRight(.2);
                while (Math.abs(Math.abs(orientation.getYaw(AngleUnit.DEGREES)) - targetAngle) > 8) {
                    orientation = imu.getRobotYawPitchRollAngles();
                    telemetry.addData("Yaw ?(Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                    telemetry.update();
                }
                 }
            case CENTER: {
                telemetry.addLine("center after start");
                telemetry.update();
                imu.resetYaw();
                targetAngle = 180;
               // everything.turnRight(.2);


                while(Math.abs(Math.abs(orientation.getYaw(AngleUnit.DEGREES))-targetAngle)>8) {
                    orientation = imu.getRobotYawPitchRollAngles();
                   // telemetry.addData("Yaw! (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                    telemetry.update();
                    //everything.turnRight(.3);
                    everything.goForward(.2);

                }
                everything.Stop();
        }
    }

  }

    public YawPitchRollAngles getAngle(){
        return imu.getRobotYawPitchRollAngles();
    }

}
