package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SkyStoneOpMode;

import java.math.BigDecimal;
import java.math.RoundingMode;

@TeleOp
@Config
public class Drive extends SkyStoneOpMode {

    private boolean slowMode = false, accelerating = false;
    private double acceleratePower = 0.0;

    @Override
    public void runOpMode() {
        initialize(false);
        actuator.setPower(0);
        reach.setPosition(0.25);
        spin.setPosition(0.2);
        clamp.setPosition(0);
        boolean lastDPLeftState = false;
        boolean lastAState1 = false;
        boolean lastAState2 = false;
        boolean lastYState1 = false;
        boolean lastBState2 = false;
        boolean lastXState1 = false;
        composeTelemetry();
        waitForStart();

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x * .9 + .1;
            double y = gamepad1.left_stick_y * .9 + .1;
            x = (x == .1) ? 0 : x;
            y = (y == .1) ? 0 : y;

            double r = Math.hypot(x, y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            double v1 = (r * Math.cos(robotAngle) + rightX) * Math.sqrt(2);
            double v2 = (r * Math.sin(robotAngle) - rightX) * Math.sqrt(2);
            double v3 = (r * Math.sin(robotAngle) + rightX) * Math.sqrt(2);
            double v4 = (r * Math.cos(robotAngle) - rightX) * Math.sqrt(2);


            if (gamepad1.dpad_down) {
                accelerating = false;
                acceleratePower = 0.0;
            }

            if (slowMode && !accelerating) {
                leftFront.setPower(v1 / 2);
                rightFront.setPower(v2 / 2);
                leftRear.setPower(v3 / 2);
                rightRear.setPower(v4 / 2);
            } else if (!accelerating) {
                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftRear.setPower(v3);
                rightRear.setPower(v4);
            }

            if (gamepad1.dpad_up) {
                accelerating = true;
                if (acceleratePower < 0.5) {
                    acceleratePower += 0.001;
                }
                rightFront.setPower(acceleratePower);
                rightRear.setPower(acceleratePower);
                leftRear.setPower(acceleratePower);
                leftFront.setPower(acceleratePower);
            }

            if (acceleratePower > 0 && !gamepad1.dpad_up) {
                acceleratePower -= 0.001;
                if (acceleratePower <= 0) {
                    acceleratePower = 0;
                    accelerating = false;
                }
                rightFront.setPower(-acceleratePower);
                rightRear.setPower(-acceleratePower);
                leftRear.setPower(-acceleratePower);
                leftFront.setPower(-acceleratePower);
            }

            double lifterPower = gamepad2.right_stick_y;
            if (downStop.getState() && lifterPower == 0) {
                lifterPower = -0.13;
            } else if (!downStop.getState()) {
                lifterPower = lifterPower > 0 ? 0 : lifterPower; // if lifterPower < 0, lifterPower = 0
            }
            lifterLeft.setPower(-lifterPower);
            lifterRight.setPower(-lifterPower);

            actuator.setPower(-gamepad2.left_stick_y);

            if (gamepad1.a && !lastAState1) {
                slowMode = !slowMode;
            }
            lastAState1 = gamepad1.a;

            if (gamepad2.a && !lastAState2) {
                lift.setPosition(round(lift.getPosition(), 2) != 0.58 ? 0.58 : 0);
            }
            lastAState2 = gamepad2.a;
            if (gamepad1.y && !lastYState1) {
                clamp.setPosition(round(clamp.getPosition(), 1) != 0.6 ? 0.6 : 0.2);
            }
            lastYState1 = gamepad1.y;

            if (gamepad1.b && !lastBState2 && (round(clamp.getPosition(), 2) != 0.6)) {
                spin.setPosition(spin.getPosition() != 0.52 ? 0.52 : 0.2);
            }
            lastBState2 = gamepad2.b;

            if (gamepad1.x && !lastXState1) {
                cap.setPosition(round(cap.getPosition(), 1) != 1.0 ? 1.0 : 0);
            }
            lastXState1 = gamepad1.x;



            if (gamepad1.dpad_left && !lastDPLeftState) {
                clamp.setPosition(0);
                lift.setPosition(0);
                reach.setPosition(0.45);

                foundationGrabberLeft.setPosition(foundationGrabberLeft.getPosition() != 0.0 ? 0.0 : 1.0);
                foundationGrabberRight.setPosition(foundationGrabberRight.getPosition() != 0.0 ? 0.0 : 1.0);
            }
            lastDPLeftState = gamepad1.dpad_left;

            telemetry.update();
        }
    }


    private void composeTelemetry() {
        telemetry.addLine().addData("slowMode", () -> slowMode);
        telemetry.addLine().addData("leftFront", () -> round(leftFront.getPower()));
        telemetry.addLine().addData("leftRear", () -> round(leftRear.getPower()));
        telemetry.addLine().addData("rightFront", () -> round(rightFront.getPower()));
        telemetry.addLine().addData("rightRear", () -> round(rightRear.getPower()));
        telemetry.addLine().addData("lifterLeft", () -> round(lifterLeft.getPower()));
        telemetry.addLine().addData("lifterRight", () -> round(lifterRight.getPower()));
        telemetry.addLine().addData("actuator", () -> round(actuator.getPower()));
        telemetry.addLine().addData("position actuator", () -> actuator.getCurrentPosition());
        telemetry.addLine().addData("foundationGrabberLeft", () -> foundationGrabberLeft.getPosition());
        telemetry.addLine().addData("foundationGrabberRight", () -> foundationGrabberRight.getPosition());
        telemetry.addLine().addData("reach", () -> round(reach.getPosition()));
        telemetry.addLine().addData("lift", () -> round(lift.getPosition()));
        telemetry.addLine().addData("clamp", () -> round(clamp.getPosition()));
        telemetry.addLine().addData("cap", () -> round(cap.getPosition()));
        telemetry.addLine().addData("spin", () -> round(spin.getPosition()));
        telemetry.addLine().addData("Down Touch Sensor pressed: ", () -> !downStop.getState());
    }

    private static double round(double value) {
        return round(value, 4);
    }

    private static double round(double value, @SuppressWarnings("SameParameterValue") int places) {
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(Double.toString(value));
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }

}
