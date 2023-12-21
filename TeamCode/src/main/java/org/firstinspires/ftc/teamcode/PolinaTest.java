package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
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


import java.util.ArrayList;


@Autonomous
public class PolinaTest extends LinearOpMode {
    OpenCvCamera camera;
    private HackHers_Lib everything;
    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;

    DcMotor ar;
    public IMU imu;
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
        //cl = hardwareMap.get(Servo.class, "cl");
        everything = new HackHers_Lib(fL, fR, bL, bR, (OpenCvWebcam) camera, ar);
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        imu.initialize(new IMU.Parameters(orientationOnRobot));


//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//                @Override
//                public void onOpened() {
//                    camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//                }
//
//                @Override
//                public void onError(int errorCode) {
//
//                }
//            });


        telemetry.setMsTransmissionInterval(50);
//        double xRotation = 0;  // enter the desired X rotation angle here.
//        double yRotation = 0;  // enter the desired Y rotation angle here.
//        double zRotation = 0;  // enter the desired Z rotation angle here.
        //Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        super.waitForStart();

        while (!isStopRequested()) {
            everything.turnRight(.3);
            //telemetry.addData("Hub orientation", "X=%.1f,  Y=%.1f,  Z=%.1f \n", xRotation, yRotation, zRotation);

            // Check to see if heading reset is requested
            if (gamepad1.y) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
            }

            // Retrieve Rotational Angles and Velocities
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
//            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
//            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
//            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.update();
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */


    }
}