package org.firstinspires.ftc.teamcode;

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
    MosaicDetectorExample.MosaicDeterminationPipeline.SkystonePosition snapshotAnalysis = MosaicDetectorExample.MosaicDeterminationPipeline.SkystonePosition.LEFT;
    private HackHers_Lib everything;
    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;
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
        everything = new HackHers_Lib(fL, fR, bL, bR, camera);
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
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
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

        while (!isStopRequested()) {

            telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
            telemetry.update();
            telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);
            imu.resetYaw();
            // Retrieve Rotational Angles
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.update();


            switch (snapshotAnalysis) {
                case LEFT: {
                    targetAngle = 180;
                    everything.turnRight(.2);
                    if(Math.abs(getTheAngle()-targetAngle)<5){
                        everything.Stop();
                    }
                    everything.Stop();
                }
                case RIGHT: {
                    everything.goForward(.3);
                    sleep(2000);
                    everything.Stop();
                }
                case CENTER: {
                    everything.goForward(.3);
                    sleep(2000);
                    everything.Stop();
                }
            }

            if (gamepad1.y) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
            }


        }
    }

    public YawPitchRollAngles getAngle(){
        return imu.getRobotYawPitchRollAngles();
    }

    public Orientation getOrientation(){
        return imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }

    public float getFirstAngleOrientation(){
        return getOrientation().firstAngle;
    }

    public float getSecondAngleOrientation() {
        return getOrientation().secondAngle;
    }
    public float getThirdAngleOrientation() {
        return getOrientation().thirdAngle;
    }

    public float getTheAngle(){
        Orientation angles = getOrientation();
        float deltaAngle = getThirdAngleOrientation() - lastAngles.thirdAngle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;
        return globalAngle;
    }
}
