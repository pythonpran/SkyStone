package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.teamcode.SkyStoneOpMode;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public abstract class AutonomousSkyStoneOpMode extends SkyStoneOpMode {

    protected OdometryGlobalCoordinatePosition globalPositionUpdate;

    protected static boolean detectedLeft = false;
    protected static boolean detectedMid = false;
    protected static boolean detectedRight = false;
    protected static SkyStoneLocation skyStoneLocation;

    private static final float rectHeight = .6f / 5f;
    private static final float rectWidth = 1.5f / 5f;

    private static final float offsetX = 0f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static final float offsetY = -3f / 16f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f / 8f + offsetX, 4f / 8f + offsetY};
    private static float[] leftPos = {2f / 8f + offsetX, 4f / 8f + offsetY};
    private static float[] rightPos = {6f / 8f + offsetX, 4f / 8f + offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private static final int rows = 640;
    private static final int cols = 480;

    protected OpenCvCamera phoneCam;


    @Override
    protected void initialize(boolean calibrateIMU) {
        super.initialize(true);
        startOpenCVCameraStream();
    }

    protected void startOpenCVCameraStream() {
        final int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);


        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages

        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
    }


    protected void createGlobalPositionUpdate() {
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(rightRear, leftFront, rightFront, ODOMETER_COUNTS_PER_INCH, 75, imu);
        globalPositionUpdate.reverseRightEncoder();
    }

    /**
     * @param speed the power
     * @param time  the millsecond to wait before running `stop`
     */
    protected void actuatormove(double speed, long time) {
        actuatormove(speed, time, () -> actuator.setPower(0));
    }

    /**
     * @param speed the power
     * @param time  the millsecond to wait before running `stop`
     * @param stop  the function to run after `time`
     */
    protected void actuatormove(double speed, long time, Runnable stop) {
        actuator.setPower(speed);
        ThreadPool.getDefaultScheduler().schedule(stop, time, TimeUnit.MILLISECONDS);
    }

    protected void move(double x, double y, double proportionX, double proportionY, double brakeDistanceY, double brakeDistanceX, double brakeVelocity) {
        y = -y;
        x = -x;
        double pX = proportionX / ODOMETER_COUNTS_PER_INCH;
        double pY = proportionY / ODOMETER_COUNTS_PER_INCH;
        double pHeading = .004;
        double targetX = -x * ODOMETER_COUNTS_PER_INCH;
        double targetY = y * ODOMETER_COUNTS_PER_INCH;
        double lastX = targetX;
        double lastY = targetY;
        while (opModeIsActive()) {
            double robotAngle = Math.atan2(targetX - globalPositionUpdate.returnXCoordinate(), targetY - globalPositionUpdate.returnYCoordinate()) + Math.PI / 4;
            // double rightX=pHeading*(globalPositionUpdate.returnOrientation());
            double rightX = pHeading * (globalPositionUpdate.returnOrientation());

            double vX = Math.abs(globalPositionUpdate.returnXCoordinate() - lastX) / ODOMETER_COUNTS_PER_INCH;
            double vY = Math.abs(globalPositionUpdate.returnYCoordinate() - lastY) / ODOMETER_COUNTS_PER_INCH;

            boolean neg = rightX > 0;
            rightX = Math.abs(rightX);
            rightX = Math.max(0.15, rightX);
            rightX = Math.min(1.0, rightX);
            rightX *= neg ? -1 : 1;
            double v1 = (Math.cos(robotAngle));
            double v2 = (Math.sin(robotAngle));
            double v3 = (Math.sin(robotAngle));
            double v4 = (Math.cos(robotAngle));
            double kY = Math.abs(targetY - globalPositionUpdate.returnYCoordinate()) * pY;
            double kX = Math.abs(targetX - globalPositionUpdate.returnXCoordinate()) * pX;

            //if statement for controlling overshoot
            if (Math.abs((targetY - globalPositionUpdate.returnYCoordinate()) / ODOMETER_COUNTS_PER_INCH) < brakeDistanceY && Math.abs(vY) > brakeVelocity) {
                v1 = 0;
                v2 = 0;
                v3 = 0;
                v4 = 0;
            }
            if (Math.abs((targetX - globalPositionUpdate.returnXCoordinate()) / ODOMETER_COUNTS_PER_INCH) < brakeDistanceX && Math.abs(vX) > brakeVelocity) {
                v1 = 0;
                v2 = 0;
                v3 = 0;
                v4 = 0;
            }
            kY = Math.max(.17, kY);
            kY = Math.min(1, kY);

            kX = Math.max(.3, kX);
            kX = Math.min(1, kX);
            if (Math.abs((targetX - globalPositionUpdate.returnXCoordinate()) / ODOMETER_COUNTS_PER_INCH) < .3 && Math.abs((targetY - globalPositionUpdate.returnYCoordinate()) / ODOMETER_COUNTS_PER_INCH) < .5) {
                brake();
                break;
            }

            double k = Math.hypot(kX, kY);
            double brakePower = 0;
            leftFront.setPower(k * v1 - rightX - brakePower);
            rightFront.setPower(k * v2 + rightX - brakePower);
            leftRear.setPower(k * v3 - rightX - brakePower);
            rightRear.setPower(k * v4 + rightX - brakePower);

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / ODOMETER_COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / ODOMETER_COUNTS_PER_INCH);
            telemetry.addData("robot angle", globalPositionUpdate.returnOrientation());
            telemetry.addData("currentVY", vY);
            telemetry.addData("xError", (targetX - globalPositionUpdate.returnXCoordinate()) / COUNTS_PER_INCH);
            telemetry.addData("yError", (targetY - globalPositionUpdate.returnYCoordinate()) / COUNTS_PER_INCH);


            telemetry.addData("Vertical left encoder position", rightRear.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", leftFront.getCurrentPosition());
            telemetry.addData("horizontal encoder position", rightFront.getCurrentPosition());
            telemetry.addData("Angle", Math.toDegrees(robotAngle));
            lastX = globalPositionUpdate.returnXCoordinate();
            lastY = globalPositionUpdate.returnYCoordinate();
            globalPositionUpdate.globalCoordinatePositionUpdate();

            telemetry.update();
        }
    }

    /**
     * Calculate the power in the x direction
     *
     * @param desiredAngle angle on the x axis
     * @param speed        robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     *
     * @param desiredAngle angle on the y axis
     * @param speed        robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int) (input.rows() * midPos[1]), (int) (input.cols() * midPos[0]));//gets value at circle
            detectedMid = (int) pixMid[0] == 0;
            skyStoneLocation = (int) pixMid[0] == 0 ? SkyStoneLocation.MIDDLE : skyStoneLocation;

            double[] pixLeft = thresholdMat.get((int) (input.rows() * leftPos[1]), (int) (input.cols() * leftPos[0]));//gets value at circle
            detectedLeft = (int) pixLeft[0] == 0;
            skyStoneLocation = (int) pixLeft[0] == 0 ? SkyStoneLocation.LEFT : skyStoneLocation;

            double[] pixRight = thresholdMat.get((int) (input.rows() * rightPos[1]), (int) (input.cols() * rightPos[0]));//gets value at circle
            detectedRight = (int) pixRight[0] == 0;
            skyStoneLocation = (int) pixRight[0] == 0 ? SkyStoneLocation.RIGHT : skyStoneLocation;

            //create three points
            Point pointMid = new Point((int) (input.cols() * midPos[0]), (int) (input.rows() * midPos[1]));
            Point pointLeft = new Point((int) (input.cols() * leftPos[0]), (int) (input.rows() * leftPos[1]));
            Point pointRight = new Point((int) (input.cols() * rightPos[0]), (int) (input.rows() * rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointLeft, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointRight, 5, new Scalar(255, 0, 0), 1);//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols() * (leftPos[0] - rectWidth / 2),
                            input.rows() * (leftPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (leftPos[0] + rectWidth / 2),
                            input.rows() * (leftPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols() * (midPos[0] - rectWidth / 2),
                            input.rows() * (midPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (midPos[0] + rectWidth / 2),
                            input.rows() * (midPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols() * (rightPos[0] - rectWidth / 2),
                            input.rows() * (rightPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (rightPos[0] + rectWidth / 2),
                            input.rows() * (rightPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport) {
                case THRESHOLD:
                    return thresholdMat;
                case detection:
                    return all;
                default:
                    return input;
            }
        }

    }

    enum SkyStoneLocation {
        LEFT, MIDDLE, RIGHT;

        @Override
        public String toString() {
            return equals(LEFT) ? "Left" : equals(MIDDLE) ? "Middle" : "Right";
        }
    }
}
