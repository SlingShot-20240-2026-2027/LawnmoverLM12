package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {

    public static String FRONT_LEFT_NAME = "fl";
    public static String FRONT_RIGHT_NAME = "fr";
    public static String BACK_LEFT_NAME = "bl";
    public static String BACK_RIGHT_NAME = "br";
    public static boolean LEFT_DRIVE_REVERSED = false;
    public static boolean RIGHT_DRIVE_REVERSED = true;

    public static String INTAKE_NAME = "intake";
    public static boolean INTAKE_REVERSED = true;
    public static double INTAKE_POWER = 1.0;
    public static double OUTTAKE_POWER = 1.0;

    public static String TRANSFER_NAME = "transfer";
    public static boolean TRANSFER_REVERSED = true;
    public static double TRANSFER_POWER = 1.0;
    public static double TRANSFER_REVERSE_POWER = 1.0;

    public static String FLYWHEEL1_NAME = "flywheel1";
    public static String FLYWHEEL2_NAME = "flywheel2";
    public static boolean FLYWHEEL1_REVERSED = true;
    public static boolean FLYWHEEL2_REVERSED = true;
    public static double FLYWHEEL_VELOCITY = 0;
    public static double FLYWHEEL_P = 0.0;
    public static double FLYWHEEL_I = 0.0;
    public static double FLYWHEEL_D = 0.0;
    public static double FLYWHEEL_F = 14.0;
    public static double FLYWHEEL_READY_TOLERANCE = 30;

    public static String HOOD_NAME = "hood";
    public static double HOOD_UP = 0.43;
    public static double HOOD_DOWN = 0.0;

    public static String GATE_NAME = "gate";
    public static double GATE_CLOSED = 0.7;
    public static double GATE_OPEN = 0.4;

    public static String DEPLOY_NAME = "deployable";
    public static double DEPLOY_UP = 0.0;
    public static double DEPLOY_DOWN = 1.0;

    public static String IMU_NAME = "imu";

    private RobotConstants() {

    }
}
