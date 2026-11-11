package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretConstants {

    public static String SERVO1_NAME = "turret1";
    public static String SERVO2_NAME = "turret2";
    public static boolean SERVO1_REVERSED = false;
    public static boolean SERVO2_REVERSED = false;
    public static double SERVO_CENTER = 0.5;
    public static double SERVO2_TRIM = 0.0;
    public static double DEGREES_PER_SERVO_UNIT = 355.0;
    public static double SERVO_MIN_POSITION = 0.0;
    public static double SERVO_MAX_POSITION = 1.0;

    public static double TURRET_MIN_DEG = -90.0;
    public static double TURRET_MAX_DEG = 90.0;
    public static double TURRET_DEG_PER_SEC = 300.0;
    public static double ON_TARGET_TOLERANCE_DEG = 2.0;

    public static double TURRET_OFFSET_FORWARD_IN = 0.0;
    public static double TURRET_OFFSET_LEFT_IN = 0.0;

    public static double TARGET1_X = 62.6;
    public static double TARGET1_Y = 72.0;
    public static double TARGET2_X = 81.4;
    public static double TARGET2_Y = 72.0;
    public static double MIN_TARGET_DISTANCE_IN = 6.0;

    public static double AIM_TRIM_STEP_DEG = 2.0;
    public static double MAX_AIM_TRIM_DEG = 20.0;
    public static double LOOKAHEAD_SEC = 0.0;
    public static double VELOCITY_FILTER = 0.5;
    public static double MANUAL_ANGLE_DEG = 0.0;

    public static long PINPOINT_SETTLE_MS = 500;

    public static boolean RESET_POSE_ON_INIT = true;
    public static double START_X = 55.92;
    public static double START_Y = 9.218;
    public static double START_HEADING_DEG = 90.0;

    public static double RELOCALIZE_X = 55.92;
    public static double RELOCALIZE_Y = 9.218;
    public static double RELOCALIZE_HEADING_DEG = 90.0;

    public enum Corner {BOTTOM_LEFT, BOTTOM_RIGHT, TOP_LEFT, TOP_RIGHT}

    public static Corner RELOCALIZE_CORNER = Corner.TOP_LEFT;
    public static double CORNER_HEADING_DEG = 90.0;
    public static double ROBOT_LENGTH_IN = 17.5;
    public static double ROBOT_WIDTH_IN = 17.5;
    public static double FIELD_SIZE_IN = 144.0;

    private TurretConstants() {
    }
}
