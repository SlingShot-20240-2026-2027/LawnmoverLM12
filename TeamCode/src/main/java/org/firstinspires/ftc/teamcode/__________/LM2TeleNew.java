package org.firstinspires.ftc.teamcode.__________;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Auton.Ivy.ButterflyDrive;
import org.firstinspires.ftc.teamcode.Auton.Ivy.Gate;
import org.firstinspires.ftc.teamcode.Auton.Ivy.Hood;
import org.firstinspires.ftc.teamcode.Auton.Ivy.HoodRegression;
import org.firstinspires.ftc.teamcode.Auton.Ivy.Intake;
import org.firstinspires.ftc.teamcode.Auton.Ivy.Ramp;
import org.firstinspires.ftc.teamcode.Auton.Ivy.Shooter;
import org.firstinspires.ftc.teamcode.Auton.Ivy.Transfer;
import org.firstinspires.ftc.teamcode.Auton.Ivy.VelocityRegression;
import org.firstinspires.ftc.teamcode.Auton.RobotConstants;
import org.firstinspires.ftc.teamcode.Auton.TurretConstants;
import org.firstinspires.ftc.teamcode.Auton.pedro.Constants;

import java.util.Locale;

@Config
@TeleOp(name = "LM2TeleNew", group = "TeleOp")
public class LM2TeleNew extends LinearOpMode {

    public static double DEADBAND = 0.05;
    public static double SLOW_SPEED = 0.35;
    public static double TRIGGER_THRESHOLD = 0.3;
    public static double VELOCITY_UPDATE_TPS = 10;
    public static double MANUAL_DISTANCE_IN = 60;
    public static double SHOOTER_OFFSET_FORWARD_IN = 0;
    public static double SHOOTER_OFFSET_LEFT_IN = 0;
    public static double SHOOTER_FACING_DEG = 0;
    public static double AIM_TOLERANCE_DEG = 3;

    private enum Target {TARGET_1, TARGET_2}

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Hood hood;
    private Gate gate;
    private Ramp ramp;
    private ButterflyDrive butterfly;
    private PinpointLocalizer localizer;
    private String localizerError = "";
    private double poseX, poseY, poseHeading;

    private final Gamepad g1 = new Gamepad();
    private final Gamepad g1Prev = new Gamepad();

    private boolean autoShot = true;
    private boolean usingDistance = false;
    private boolean shooting = false;
    private boolean wasReady = false;
    private boolean deployed = false;
    private Target target = Target.TARGET_1;

    private Intake.State appliedIntake = null;
    private Transfer.State appliedTransfer = null;
    private Gate.State appliedGate = null;
    private double appliedVelocity = Double.NaN;

    private double distanceIn = Double.NaN;
    private double aimErrorDeg = Double.NaN;
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
        butterfly = new ButterflyDrive(hardwareMap);

        setGate(Gate.State.CLOSED);
        hood.setPosition(HoodRegression.positionFor(MANUAL_DISTANCE_IN));
        ramp.setState(Ramp.State.UP);
        butterfly.setMode(ButterflyDrive.Mode.MECANUM);

        try {
            localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        } catch (RuntimeException e) {
            localizer = null;
            localizerError = e.getClass().getSimpleName();
        }

        if (localizer != null) {
            telemetry.addLine("Starting Pinpoint - keep the robot still...");
            telemetry.update();
            sleep(Math.max(0, TurretConstants.PINPOINT_SETTLE_MS));
            if (TurretConstants.RESET_POSE_ON_INIT) {
                setPose(TurretConstants.START_X, TurretConstants.START_Y, TurretConstants.START_HEADING_DEG);
            }
        }

        while (opModeInInit()) {
            updatePose();
            telemetry.addLine("LM2Tele ready. Press PLAY.");
            telemetry.addLine("Intake, transfer and flywheel start at PLAY.");
            telemetry.addData("Pinpoint", localizer != null ? "OK" : "NOT FOUND (" + localizerError + ") - fixed shot only");
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
            updateAim();
            applyShot();
            applyMechanisms();
            readyRumble();
            addTelemetry();
        }

        stopAll();
    }

    private void drive() {
        double y = deadband(-g1.left_stick_y);
        double x = canStrafe() ? deadband(g1.left_stick_x) : 0;
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

    private boolean canStrafe() {
        return butterfly.getMode() == ButterflyDrive.Mode.MECANUM && butterfly.isSettled();
    }

    private void buttons() {
        if (pressed(g1.a, g1Prev.a)) {
            boolean traction = !butterfly.isTraction();
            butterfly.setMode(traction ? ButterflyDrive.Mode.TRACTION : ButterflyDrive.Mode.MECANUM);
            gamepad1.rumbleBlips(traction ? 2 : 1);
        }
        if (pressed(g1.left_bumper, g1Prev.left_bumper)) {
            deployed = !deployed;
            ramp.setState(deployed ? Ramp.State.DEPLOY : Ramp.State.UP);
            gamepad1.rumbleBlips(deployed ? 2 : 1);
        }
        if (pressed(g1.y, g1Prev.y)) {
            autoShot = !autoShot;
            gamepad1.rumbleBlips(autoShot ? 2 : 1);
        }
        if (pressed(g1.b, g1Prev.b)) {
            target = target == Target.TARGET_1 ? Target.TARGET_2 : Target.TARGET_1;
            gamepad1.rumbleBlips(target == Target.TARGET_1 ? 1 : 2);
        }
        if (pressed(g1.back, g1Prev.back) && localizer != null) {
            relocalizeInCorner();
            gamepad1.rumble(300);
        }
        shooting = g1.right_trigger > TRIGGER_THRESHOLD;
    }

    private void updateAim() {
        if (localizer == null) {
            distanceIn = Double.NaN;
            aimErrorDeg = Double.NaN;
            return;
        }
        double c = Math.cos(poseHeading);
        double s = Math.sin(poseHeading);
        double sx = poseX + SHOOTER_OFFSET_FORWARD_IN * c - SHOOTER_OFFSET_LEFT_IN * s;
        double sy = poseY + SHOOTER_OFFSET_FORWARD_IN * s + SHOOTER_OFFSET_LEFT_IN * c;
        double tx = target == Target.TARGET_1 ? TurretConstants.TARGET1_X : TurretConstants.TARGET2_X;
        double ty = target == Target.TARGET_1 ? TurretConstants.TARGET1_Y : TurretConstants.TARGET2_Y;
        double dx = tx - sx;
        double dy = ty - sy;
        distanceIn = Math.hypot(dx, dy);

        double shooterHeading = poseHeading + Math.toRadians(SHOOTER_FACING_DEG);
        aimErrorDeg = Math.toDegrees(wrap(Math.atan2(dy, dx) - shooterHeading));
    }

    private void applyShot() {
        usingDistance = autoShot && !Double.isNaN(distanceIn);
        double shotDistance = usingDistance ? distanceIn : MANUAL_DISTANCE_IN;

        targetVelocity = VelocityRegression.velocityFor(shotDistance);
        targetHood = HoodRegression.positionFor(shotDistance);
        hood.setPosition(targetHood);

        if (Double.isNaN(appliedVelocity) || Math.abs(targetVelocity - appliedVelocity) >= VELOCITY_UPDATE_TPS) {
            shooter.spin(targetVelocity);
            appliedVelocity = targetVelocity;
        }
    }

    private void applyMechanisms() {
        if (g1.dpad_down) {
            setIntake(Intake.State.OUTTAKE);
            setTransfer(Transfer.State.REVERSE);
            setGate(Gate.State.CLOSED);
        } else {
            setIntake(Intake.State.INTAKE);
            setTransfer(Transfer.State.TRANSFER);
            setGate(shooting ? Gate.State.OPEN : Gate.State.CLOSED);
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
        butterfly.setMode(ButterflyDrive.Mode.MECANUM);
    }

    private void addTelemetry() {
        telemetry.addData("Shot", String.format(Locale.US, "%s  %.0f tps  hood %.3f",
                usingDistance ? "AUTO" : (autoShot ? "FIXED (no Pinpoint)" : "FIXED"),
                targetVelocity, targetHood));
        telemetry.addData("Flywheel", shooter.isReady() ? "READY" : "spinning up");
        telemetry.addData("Target", target);
        telemetry.addData("Distance (in)", Double.isNaN(distanceIn) ? "-" : String.format(Locale.US, "%.1f", distanceIn));
        if (Double.isNaN(aimErrorDeg)) {
            telemetry.addData("Aim", "-");
        } else if (Math.abs(aimErrorDeg) <= AIM_TOLERANCE_DEG) {
            telemetry.addData("Aim", "ON TARGET (%.1f deg)", aimErrorDeg);
        } else {
            telemetry.addData("Aim", "turn %s %.0f deg", aimErrorDeg > 0 ? "LEFT" : "RIGHT", Math.abs(aimErrorDeg));
        }
        telemetry.addData("Shooting", shooting ? "YES" : "no");
        telemetry.addData("Feed", g1.dpad_down ? "REVERSE (unjam)" : "intake + transfer ON");
        telemetry.addData("Deployable", deployed ? "DOWN" : "UP");
        telemetry.addData("Drive", butterfly.isTraction()
                ? (butterfly.isSettled() ? "TRACTION (no strafe)" : "TRACTION (lowering)")
                : (butterfly.isSettled() ? "MECANUM" : "MECANUM (lifting, no strafe)"));
        telemetry.addData("Slow mode", g1.left_trigger > TRIGGER_THRESHOLD ? "ON" : "off");
        telemetry.addData("Pose", localizer == null ? "Pinpoint not found"
                : String.format(Locale.US, "x %.1f  y %.1f  h %.1f", poseX, poseY, Math.toDegrees(poseHeading)));
        telemetry.addLine();
        telemetry.addLine("RT shoot | LT slow | LB deployable | A butterfly | D-pad down unjam");
        telemetry.addLine("Y auto/fixed shot | B target 1/2 | BACK corner reset");
        shooter.addTelemetry(telemetry);
        telemetry.update();
    }

    private void setPose(double x, double y, double headingDeg) {
        if (localizer == null) return;
        localizer.setPose(new Pose(x, y, Math.toRadians(headingDeg)));
        poseX = x;
        poseY = y;
        poseHeading = Math.toRadians(headingDeg);
    }

    private void updatePose() {
        if (localizer == null) return;
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

    private static double wrap(double a) {
        a = a % (2 * Math.PI);
        if (a > Math.PI) a -= 2 * Math.PI;
        if (a <= -Math.PI) a += 2 * Math.PI;
        return a;
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