package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
@SuppressWarnings("WeakerAccess")
public class ServoTester extends SkyStoneOpMode {

    public static double reachPosition = 0.25;
    public static double liftPosition = 0.58; // goes to 0.0
    public static double clampPosition = 0.55;
    public static double capPosition = 0.9;
    public static double spinPosition = 0.52;
    public static double foundationGrabberLeftPosition = 0.0;
    public static double foundationGrabberRightPosition = 1.0;

    @Override
    public void runOpMode() {
        initialize(false);
        composeTelemetry();
        while (!isStopRequested()) {
            foundationGrabberLeft.setPosition(foundationGrabberLeftPosition);
            foundationGrabberRight.setPosition(foundationGrabberRightPosition);
            reach.setPosition(reachPosition);
            lift.setPosition(liftPosition);
            clamp.setPosition(clampPosition);
            cap.setPosition(capPosition); // it goes down to 0.6
            spin.setPosition(spinPosition);
            telemetry.update();
        }
    }

    private void composeTelemetry() {
        telemetry.addLine().addData("foundationGrabberLeft", () -> foundationGrabberLeft.getPosition());
        telemetry.addLine().addData("foundationGrabberRight", () -> foundationGrabberRight.getPosition());
        telemetry.addLine().addData("reach", () -> reach.getPosition());
        telemetry.addLine().addData("lift", () -> lift.getPosition());
        telemetry.addLine().addData("clamp", () -> clamp.getPosition());
        telemetry.addLine().addData("cap", () -> cap.getPosition());
        telemetry.addLine().addData("spin", () -> spin.getPosition());
        telemetry.update();
    }
}
