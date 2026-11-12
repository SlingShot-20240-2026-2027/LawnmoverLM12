package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Ivy.Gate;
import org.firstinspires.ftc.teamcode.Ivy.Hood;
import org.firstinspires.ftc.teamcode.Ivy.HoodRegression;
import org.firstinspires.ftc.teamcode.Ivy.Intake;
import org.firstinspires.ftc.teamcode.Ivy.Ramp;
import org.firstinspires.ftc.teamcode.Ivy.Shooter;
import org.firstinspires.ftc.teamcode.Ivy.Transfer;
import org.firstinspires.ftc.teamcode.Ivy.Turret;
import org.firstinspires.ftc.teamcode.Ivy.VelocityRegression;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.TurretConstants;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@TeleOp(name = "LM2TeleTurret", group = "TeleOp")
public class LM2TeleTurret extends LinearOpMode {

    public static double DEADBAND = 0.05;
    public static double SLOW_SPEED = 0.35;
    public static double TRIGGER_THRESHOLD = 0.3;
    public static double VELOCITY_UPDATE_TPS = 10;
    public static double MIN_VELOCITY = 600;
    public static double MAX_VELOCITY = 2000;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Hood hood;
    private Gate gate;
    private Ramp ramp;
    private Turret turret;
    private PinpointLocalizer localizer;
    private double poseX, poseY, poseHeading;

    private final Gamepad g1 = new Gamepad();
    private final Gamepad g1Prev = new Gamepad();

    private boolean intakeOn = true;
    private boolean shooting = false;
    private boolean wasReady = false;
    private boolean autoShot = true;
    private boolean usingRegression = false;
    private Hood.State manualHood = Hood.State.UP;
    private boolean deployed = false;

    private Intake.State appliedIntake = null;
    private Transfer.State appliedTransfer = null;
    private Gate.State appliedGate = null;
    private double appliedVelocity = Double.NaN;
    private double targetVelocity = 0;
    private double targetHood = 0;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotorEx.class, RobotConstants.FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotorEx.class, RobotConstants.FRONT_RIGHT_NAME);
        backLeft = hardwareMap.get(DcMotorEx.class, RobotConstants.BACK_LEFT_NAME);
        backRight = hardwareMap.get(DcMotorEx.class, RobotConstants.BACK_RIGHT_NAME);
        frontLeft.setDirection(direction(RobotConstants.LEFT_DRIVE_REVERSED));
        backLeft.setDirection(direction(RobotConstants.LEFT_DRIVE_REVERSED));
        frontRight.setDirection(direction(RobotConstants.RIGHT_DRIVE_REVERSED));
        backRight.setDirection(direction(RobotConstants.RIGHT_DRIVE_REVERSED));
        for (DcMotorEx m : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setPower(0);
        }

        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        hood = new Hood(hardwareMap);
        gate = new Gate(hardwareMap);
        ramp = new Ramp(hardwareMap);
        turret = new Turret(hardwareMap);
//        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);

        setGate(Gate.State.CLOSED);
        hood.setState(manualHood);
        ramp.setState(Ramp.State.UP);

        telemetry.addLine("Starting Pinpoint - keep the robot still...");
        telemetry.update();
        sleep(Math.max(0, TurretConstants.PINPOINT_SETTLE_MS));
        if (TurretConstants.RESET_POSE_ON_INIT) {
            setPose(TurretConstants.START_X, TurretConstants.START_Y, TurretConstants.START_HEADING_DEG);
        }
        updatePose();

        while (opModeInInit()) {
            updatePose();
            telemetry.addLine("LM2TeleTurret ready. Press PLAY.");
            telemetry.addLine("Intake and flywheel start at PLAY.");
            telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f", poseX, poseY, Math.toDegrees(poseHeading));
            telemetry.addData("Hood regression", HoodRegression.describe());
            telemetry.addData("Velocity regression", VelocityRegression.describe());
            telemetry.update();
            idle();
        }
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            g1Prev.copy(g1);
            g1.copy(gamepad1);

            updatePose();
            drive();
            buttons();
            turret.update(poseX, poseY, poseHeading);
            applyShot();
            applyMechanisms();
            readyRumble();
            addTelemetry();
        }

        stopAll();
    }

    private void drive() {
        double y = deadband(-g1.left_stick_y);
        double x = deadband(g1.left_stick_x);
        double rx = deadband(g1.right_stick_x);
        double speed = g1.left_trigger > TRIGGER_THRESHOLD ? SLOW_SPEED : 1.0;

        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));

        frontLeft.setPower(fl / max * speed);
        frontRight.setPower(fr / max * speed);
        backLeft.setPower(bl / max * speed);
        backRight.setPower(br / max * speed);
    }

    private void buttons() {
        if (pressed(g1.right_bumper, g1Prev.right_bumper)) {
            intakeOn = !intakeOn;
            gamepad1.rumbleBlips(intakeOn ? 2 : 1);
        }
        if (pressed(g1.dpad_up, g1Prev.dpad_up)) {
            autoShot = true;
            gamepad1.rumbleBlips(2);
        }
        if (pressed(g1.dpad_down, g1Prev.dpad_down)) {
            if (autoShot) {
                autoShot = false;
            } else {
                manualHood = manualHood == Hood.State.UP ? Hood.State.DOWN : Hood.State.UP;
            }
            gamepad1.rumbleBlips(1);
        }
        if (pressed(g1.x, g1Prev.x)) {
            deployed = !deployed;
            ramp.setState(deployed ? Ramp.State.DEPLOY : Ramp.State.UP);
            gamepad1.rumbleBlips(deployed ? 2 : 1);
        }
        if (pressed(g1.y, g1Prev.y)) {
            turret.toggleMode();
            gamepad1.rumbleBlips(turret.getMode() == Turret.Mode.AUTO_AIM ? 2 : 1);
        }
        if (pressed(g1.b, g1Prev.b)) {
            turret.toggleTarget();
            gamepad1.rumbleBlips(turret.getTarget() == Turret.Target.TARGET_1 ? 1 : 2);
        }
        if (pressed(g1.dpad_left, g1Prev.dpad_left)) {
            turret.addTrim(TurretConstants.AIM_TRIM_STEP_DEG);
        }
        if (pressed(g1.dpad_right, g1Prev.dpad_right)) {
            turret.addTrim(-TurretConstants.AIM_TRIM_STEP_DEG);
        }
        if (pressed(g1.a, g1Prev.a)) {
            turret.resetTrim();
            gamepad1.rumbleBlips(1);
        }
        if (pressed(g1.back, g1Prev.back)) {
            relocalizeInCorner();
            gamepad1.rumble(300);
        }
        shooting = g1.right_trigger > TRIGGER_THRESHOLD;
    }

    private void applyShot() {
        double distance = turret.getDistanceIn();
        usingRegression = autoShot
                && turret.getMode() == Turret.Mode.AUTO_AIM
                && !Double.isNaN(distance);

        if (usingRegression) {
            targetVelocity = VelocityRegression.velocityFor(distance);
            targetHood = HoodRegression.positionFor(distance);
            hood.setPosition(targetHood);
        } else {
            targetVelocity = RobotConstants.FLYWHEEL_VELOCITY;
            targetHood = manualHood == Hood.State.UP ? RobotConstants.HOOD_UP : RobotConstants.HOOD_DOWN;
            if (hood.getState() != manualHood || Math.abs(hood.getPosition() - targetHood) > 0.001) {
                hood.setState(manualHood);
            }
        }

        double velocity = Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, targetVelocity));
        if (Double.isNaN(appliedVelocity) || Math.abs(velocity - appliedVelocity) >= VELOCITY_UPDATE_TPS) {
            shooter.spin(velocity);
            appliedVelocity = velocity;
        }
    }

    private void applyMechanisms() {
        boolean unjam = g1.left_bumper;

        if (unjam) {
            setIntake(Intake.State.OUTTAKE);
            setTransfer(Transfer.State.REVERSE);
            setGate(Gate.State.CLOSED);
        } else if (shooting) {
            setIntake(Intake.State.INTAKE);
            setTransfer(Transfer.State.TRANSFER);
            setGate(Gate.State.OPEN);
        } else {
            setIntake(intakeOn ? Intake.State.INTAKE : Intake.State.IDLE);
            setTransfer(Transfer.State.TRANSFER);
            setGate(Gate.State.CLOSED);
        }
    }

    private void readyRumble() {
        boolean ready = shooter.isReady();
        if (ready && !wasReady) {
            gamepad1.rumbleBlips(2);
        }
        wasReady = ready;
    }

    private void relocalizeInCorner() {
        double h = Math.toRadians(TurretConstants.CORNER_HEADING_DEG);
        double halfLength = Math.abs(TurretConstants.ROBOT_LENGTH_IN) / 2.0;
        double halfWidth = Math.abs(TurretConstants.ROBOT_WIDTH_IN) / 2.0;
        double halfX = Math.abs(Math.cos(h)) * halfLength + Math.abs(Math.sin(h)) * halfWidth;
        double halfY = Math.abs(Math.sin(h)) * halfLength + Math.abs(Math.cos(h)) * halfWidth;
        TurretConstants.Corner corner = TurretConstants.RELOCALIZE_CORNER;
        boolean left = corner == TurretConstants.Corner.BOTTOM_LEFT || corner == TurretConstants.Corner.TOP_LEFT;
        boolean bottom = corner == TurretConstants.Corner.BOTTOM_LEFT || corner == TurretConstants.Corner.BOTTOM_RIGHT;
        double x = left ? halfX : TurretConstants.FIELD_SIZE_IN - halfX;
        double y = bottom ? halfY : TurretConstants.FIELD_SIZE_IN - halfY;
        setPose(x, y, TurretConstants.CORNER_HEADING_DEG);
    }

    private void setIntake(Intake.State s) {
        if (s != appliedIntake) {
            intake.setState(s);
            appliedIntake = s;
        }
    }

    private void setTransfer(Transfer.State s) {
        if (s != appliedTransfer) {
            transfer.setState(s);
            appliedTransfer = s;
        }
    }

    private void setGate(Gate.State s) {
        if (s != appliedGate) {
            gate.setState(s);
            appliedGate = s;
        }
    }

    private void stopAll() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        shooter.stop();
        intake.setState(Intake.State.IDLE);
        transfer.setState(Transfer.State.IDLE);
        gate.setState(Gate.State.CLOSED);
        ramp.setState(Ramp.State.UP);
    }

    private void addTelemetry() {
        telemetry.addData("Shot", usingRegression
                ? String.format("AUTO  %.0f tps  hood %.3f", targetVelocity, targetHood)
                : String.format("MANUAL  %.0f tps  hood %s%s", targetVelocity, manualHood,
                autoShot ? "  (auto waiting for turret aim)" : ""));
        telemetry.addData("Flywheel", shooter.isReady() ? "READY" : "spinning up");
        telemetry.addData("Shooting", shooting ? "YES" : "no");
        telemetry.addData("Intake", g1.left_bumper ? "REVERSE" : (intakeOn ? "ON" : "OFF"));
        telemetry.addData("Transfer", g1.left_bumper ? "REVERSE (unjam)" : "ON");
        telemetry.addData("Deployable", deployed ? "DOWN" : "UP");
        telemetry.addData("Slow mode", g1.left_trigger > TRIGGER_THRESHOLD ? "ON" : "off");
        turret.addTelemetry(telemetry);
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f", poseX, poseY, Math.toDegrees(poseHeading));
        telemetry.addLine();
        telemetry.addLine("RT shoot | RB intake | LB unjam | LT slow | X deployable");
        telemetry.addLine("D-pad up auto shot | D-pad down manual hood UP/DOWN");
        telemetry.addLine("Y turret auto/manual | B target 1/2 | D-pad L/R aim trim | A reset trim | BACK corner reset");
        shooter.addTelemetry(telemetry);
        telemetry.update();
    }

    private void setPose(double x, double y, double headingDeg) {
        localizer.setPose(new Pose(x, y, Math.toRadians(headingDeg)));
        poseX = x;
        poseY = y;
        poseHeading = Math.toRadians(headingDeg);
    }

    private void updatePose() {
        localizer.update();
        Pose pose = localizer.pose();
        if (pose == null) return;
        double x = pose.x();
        double y = pose.y();
        double h = pose.heading();
        if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(h)) return;
        poseX = x;
        poseY = y;
        poseHeading = h;
    }

    private static DcMotorSimple.Direction direction(boolean reversed) {
        return reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }

    private static boolean pressed(boolean now, boolean before) {
        return now && !before;
    }

    private static double deadband(double v) {
        return Math.abs(v) < DEADBAND ? 0 : v;
    }
}
