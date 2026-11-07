package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "SimpleTeleOp")
public class SimpleTeleop extends OpMode {

    /** Mec Define **/
    DcMotor leftFront, rightFront, leftBack, rightBack;

    /** Intake + Transfer **/
    private DcMotor intake, transfer;

    /** Shooter Flywheels **/
    private DcMotorEx flywheel1, flywheel2;
    public static final double FLYWHEEL_VELOCITY = 1150;      // ticks per second (shooting)
    public static final double FLYWHEEL_IDLE_VELOCITY = 1150; // ticks per second (idle)

    // Velocity PIDF. Set F = 32767 / maxTicksPerSec (measure max at full power), then tune P.
    public static final double FLYWHEEL_P = 1.0;
    public static final double FLYWHEEL_I = 0.0;
    public static final double FLYWHEEL_D = 0.01;
    public static final double FLYWHEEL_F = 17.6;

    /** Gate **/
    private Servo gate;
    public static final double GATE_OUT = 0.66; // resting position (outward)
    public static final double GATE_IN  = 0.4;  // shooting position (inward)

    /** Hood **/
    private Servo hood;
    public static final double HOOD_POSITION = 0.4; // fixed hood position

    /** Deployable (toggle) **/
    private Servo deployable;
    public static final double DEPLOY_UP   = 0.0; // tune
    public static final double DEPLOY_DOWN = 1; // tune
    private boolean deployed = false;
    private boolean lastLeftBumper = false;

    /** RGB Indicator Light **/
//    private Servo light;
//    public static final double LIGHT_OFF   = 0.0;
//    public static final double LIGHT_GREEN = 0.5;

    /** Color Sensor (full-intake detection) **/
//    private NormalizedColorSensor colorSensor;
//    private final ElapsedTime yellowTimer = new ElapsedTime();
//    private boolean yellowWasSeen = false;
//    public static final double YELLOW_HUE_MIN = 40;   // tune with telemetry
//    public static final double YELLOW_HUE_MAX = 80;   // tune with telemetry
//    public static final double YELLOW_TIME_SEC = 1.0;

    @Override
    public void init() {
        /** Mec Init **/
        leftFront  = hardwareMap.get(DcMotor.class, "fl");
        rightFront = hardwareMap.get(DcMotor.class, "fr");
        leftBack   = hardwareMap.get(DcMotor.class, "bl");
        rightBack  = hardwareMap.get(DcMotor.class, "br");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        /** Intake / Transfer Init **/
        intake   = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        // Must come AFTER the hardwareMap.get calls above
        intake.setDirection(DcMotor.Direction.REVERSE);
        transfer.setDirection(DcMotor.Direction.REVERSE);

        /** Flywheel Init **/
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        // Both reversed = both spin the same way. Reverse only one if they're mounted mirrored.
        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);

        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply the same PIDF to BOTH flywheels
        PIDFCoefficients pidf = new PIDFCoefficients(
                FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        /** Gate Init **/
        gate = hardwareMap.get(Servo.class, "gate");
        gate.setPosition(GATE_OUT);

        /** Hood Init **/
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(HOOD_POSITION);

        /** Deployable Init **/
        deployable = hardwareMap.get(Servo.class, "3");
        deployable.setPosition(DEPLOY_UP);

        /** Color Init **/
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "revColorV3");
//        colorSensor.setGain(11);

        /** Light Init **/
//        light = hardwareMap.get(Servo.class, "light");
//        light.setPosition(LIGHT_OFF);
    }

    @Override
    public void loop() {
        /** ---------------- Mecanum (diamond / O pattern) ---------------- **/
        double drive  = -gamepad1.left_stick_y;
        double strafe =  -gamepad1.left_stick_x;
        double turn   =  gamepad1.right_stick_x;

        // Strafe signs are flipped vs. an X-pattern drivetrain
        double leftFrontPower  = drive - strafe + turn;
        double rightFrontPower = drive + strafe - turn;
        double leftBackPower   = drive + strafe + turn;
        double rightBackPower  = drive - strafe - turn;

        double max = Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
        );

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        // Hood held at a fixed position (0.7)
        hood.setPosition(HOOD_POSITION);

        /** ---------------- Intake (right bumper) & Transfer (always on) ---------------- **/
        intake.setPower(gamepad1.right_bumper ? 1.0 : 0);

        if(gamepad1.left_trigger >0.5){
            transfer.setPower(-1);
        }
        else{
            transfer.setPower(1);
        }






        /** ---------------- Shooter Flywheels & Gate ---------------- **/
        if (gamepad1.right_trigger > 0.5) {
            flywheel1.setVelocity(FLYWHEEL_VELOCITY);
            flywheel2.setVelocity(FLYWHEEL_VELOCITY);
            gate.setPosition(GATE_IN);
        } else {
            flywheel1.setVelocity(FLYWHEEL_IDLE_VELOCITY);
            flywheel2.setVelocity(FLYWHEEL_IDLE_VELOCITY);
            gate.setPosition(GATE_OUT);
        }

        /** ---------------- Deployable (left bumper toggles down/up) ---------------- **/
        if (gamepad1.left_bumper ) {
            deployable.setPosition(DEPLOY_DOWN);
        }else{
            deployable.setPosition(DEPLOY_UP);
        }




        telemetry.addData("Deployed", deployed);
        telemetry.addData("Flywheel 1 vel", flywheel1.getVelocity());
        telemetry.addData("Flywheel 2 vel", flywheel2.getVelocity());
    }
}