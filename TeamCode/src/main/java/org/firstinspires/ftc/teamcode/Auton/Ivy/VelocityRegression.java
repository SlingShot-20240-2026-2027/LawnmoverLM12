package org.firstinspires.ftc.teamcode.Auton.Ivy;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Auton.RobotConstants;

@Config
public class VelocityRegression {

    public static String DATA = "30:1350, 60:1350, 90:1350";
    public static int DEGREE = 2;
    public static double MIN_VELOCITY = 600;
    public static double MAX_VELOCITY = 2000;

    private static final PolyFit FIT = new PolyFit();

    private VelocityRegression() {
    }

    public static double velocityFor(double distanceIn) {
        double value = FIT.evaluate(DATA, DEGREE, distanceIn);
        if (Double.isNaN(value)) {
            value = RobotConstants.FLYWHEEL_VELOCITY;
        }
        double lo = Math.min(MIN_VELOCITY, MAX_VELOCITY);
        double hi = Math.max(MIN_VELOCITY, MAX_VELOCITY);
        return Range.clip(value, lo, hi);
    }

    public static boolean isValid() {
        return !Double.isNaN(FIT.evaluate(DATA, DEGREE, 0));
    }

    public static String describe() {
        return FIT.describe(DATA, DEGREE);
    }
}
