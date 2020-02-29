package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


/**
 * Created by maryjaneb  on 11/13/2016.
 * <p>
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 * <p>
 * monitor: 640 x 480
 * YES
 */
@Autonomous
public class OpenCVSkyStoneDetector extends AutonomousSkyStoneOpMode {

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        OpenCvCamera phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);


        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new AutonomousSkyStoneOpMode.StageSwitchingPipeline());//different stages

        sleep(500);

        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);//display on RC

        while (!isStopRequested()) {
            telemetry.addData("Values", detectedLeft + "   " + detectedMid + "   " + detectedRight);
            if (detectedLeft && !detectedMid && !detectedRight) {
                telemetry.addData("Location", "Left");
            } else if (detectedMid && !detectedLeft && !detectedRight) {
                telemetry.addData("Location", "Middle");
            } else if (detectedRight && !detectedMid && !detectedLeft) {
                telemetry.addData("Location", "Right");
            } else {
                telemetry.addData("Location", "Not visible");
            }
            telemetry.update();
        }
    }

}