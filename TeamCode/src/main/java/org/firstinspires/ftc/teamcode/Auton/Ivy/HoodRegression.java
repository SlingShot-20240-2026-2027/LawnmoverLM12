package org.firstinspires.ftc.teamcode.Auton.Ivy;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Auton.RobotConstants;

@Config
public class HoodRegression {

    public static String DATA = "30:0.43, 60:0.43, 90:0.43";
    public static int DEGREE = 2;
    public static double MIN_POSITION = 0.0;
    public static double MAX_POSITION = 1.0;

    private static final PolyFit FIT = new PolyFit();

    private HoodRegression() {
    }

    public static double positionFor(double distanceIn) {
        double value = FIT.evaluate(DATA, DEGREE, distanceIn);
        if (Double.isNaN(value)) {
            value = RobotConstants.HOOD_UP;
        }
        double lo = Math.min(MIN_POSITION, MAX_POSITION);
        double hi = Math.max(MIN_POSITION, MAX_POSITION);
        return Range.clip(value, lo, hi);
    }

    public static boolean isValid() {
        return !Double.isNaN(FIT.evaluate(DATA, DEGREE, 0));
    }

    public static String describe() {
        return FIT.describe(DATA, DEGREE);
    }
}
