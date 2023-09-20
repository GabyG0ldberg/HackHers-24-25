package org.firstinspires.ftc.teamcode;

import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous
public class PropDetection extends LinearOpMode {
    //OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
    WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
    OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

    camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
    {
        @Override
        public void onOpened();
        {
            // Usually this is where you'll want to start streaming from the camera (see section 4)
        }
        @Override
        public void onError(int errorCode);
        {
            /*
             * This will be called if the camera could not be opened
             */
        }
    });



    @Override
    public void runOpMode() throws InterruptedException {

    }
}
