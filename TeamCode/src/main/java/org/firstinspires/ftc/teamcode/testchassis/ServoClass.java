package org.firstinspires.ftc.teamcode.testchassis;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.math.BigDecimal;
import java.math.RoundingMode;

@Autonomous
public class ServoClass extends TestChassisOpMode {

    boolean lastAState1 = false;
    boolean lastBState2 = false;
    boolean lastBState1 = false;

    public void runOpMode(){
        initialize();
        waitForStart();

        while(opModeIsActive()) {
            if (gamepad2.a && !lastBState1) {
                clamp.setPosition(!(clamp.getPosition() <= 0.13) ? 0.1 : 1.0);
            }
            lastBState1 = gamepad2.a;

            if (gamepad2.b && !lastBState2) {
                spin.setPosition(!(spin.getPosition() <= 0.13) ? 0.1 : 1.0);
            }
            lastBState2 = gamepad2.b;

            if (gamepad2.y && !lastAState1) {
                yah.setPosition(!(yah.getPosition() <= 0.13) ? 0.1 : 1.0);
            }
            lastAState1 = gamepad2.y;

            telemetry.addData("clamp", round(clamp.getPosition(), 6));
            telemetry.addData("spin", spin.getPosition());
            telemetry.addData("yah", yah.getPosition());
            telemetry.update();
        }

    }

    private static double round(double value, @SuppressWarnings("SameParameterValue") int places) {
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(Double.toString(value));
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }
}
