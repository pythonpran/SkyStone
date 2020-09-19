package org.firstinspires.ftc.teamcode.testchassis.PurePursuit;

import org.firstinspires.ftc.teamcode.testchassis.TestChassisOpMode;

public class MathFunctions{

    public static double AngleWrap(double angle){
        while(angle < -Math.PI){
            angle += 2*Math.PI;
        }
        while(angle > Math.PI){
            angle -= 2*Math.PI;
        }
        return angle;
    }
    // Returns Degree in 360 Degree format (Compared to -180 to 180 degreee format
    public static double ImuDegree360(double angleDegree){
        return Math.toRadians((360.0+angleDegree) % 360);
    }

}
