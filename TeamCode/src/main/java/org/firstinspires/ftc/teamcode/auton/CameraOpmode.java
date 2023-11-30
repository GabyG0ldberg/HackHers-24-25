package org.firstinspires.ftc.teamcode.auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class CameraOpmode {
    @Autonomous(name = "Auto: Prop Detector")
    public class AutoMode extends LinearOpMode {
        // Handle hardware stuff...

        int width = 320;
        int height = 240;
        // store as variable here so we can access the location
        PropDetectorThePipeLine detector = new PropDetectorThePipeLine(width);
        OpenCvCamera camera;

        @Override
        public void runOpMode() {
            // robot logic...

            // https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/InternalCameraExample.java
            // Initialize the back-facing camera
           // int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
            // Connect to the camera
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {

                    camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });
            // Use the SkystoneDetector pipeline
            // processFrame() will be called to process the frame
            camera.setPipeline(detector);
            // Remember to change the camera rotation
            camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);

            //...

            PropDetectorThePipeLine.PropLocation location = detector.getLocation();
            if (location != PropDetectorThePipeLine.PropLocation.NONE) {
                // Move to the left / right
            } else {
                // Grab the skystone
            }

            //return null;
        }
    }
}
