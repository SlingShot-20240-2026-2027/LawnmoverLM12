package org.firstinspires.ftc.teamcode.__________;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auton.Ivy.Gate;
import org.firstinspires.ftc.teamcode.Auton.Ivy.Hood;
import org.firstinspires.ftc.teamcode.Auton.Ivy.Intake;
import org.firstinspires.ftc.teamcode.Auton.Ivy.Shooter;
import org.firstinspires.ftc.teamcode.Auton.Ivy.Transfer;
import org.firstinspires.ftc.teamcode.Auton.RobotConstants;

import java.util.Locale;

@Config
@TeleOp(name = "ShooterTuner", group = "TeleOp")
public class ShooterTuner extends LinearOpMode {

    public static double TRIGGER_THRESHOLD = 0.3;
    public static double STEP_TEST_DELTA = 300;
    public static long STEP_TEST_PERIOD_MS = 1500;
    public static double SHOT_DROP_TPS = 40;
    public static double MAX_VELOCITY = 2800;
    public static double DEFAULT_VELOCITY = 1350;
    public static double MIN_TUNE_VELOCITY = 200;

    private enum Param {P, D, F, VELOCITY, HOOD}

    private static final double[] BASE_STEP = {0.5, 0.1, 0.5, 25, 0.01};

    private Shooter shooter;
    private Hood hood;
    private Intake intake;
    private Transfer transfer;
    private Gate gate;
    private Telemetry dash;

    private final Gamepad g1 = new Gamepad();
    private final Gamepad g1Prev = new Gamepad();
    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime shotTimer = new ElapsedTime();

    private Param selected = Param.F;
    private final double[] stepScale = {1, 1, 1, 1, 1};
    private boolean flywheelOn = true;
    private boolean stepTest = false;
    private boolean stepHigh = true;
    private boolean wasReady = false;

    private double appliedP = Double.NaN, appliedI = Double.NaN, appliedD = Double.NaN, appliedF = Double.NaN;
    private double appliedVelocity = Double.NaN;

    private boolean inDip = false;
    private double dipMin = 0;
    private double lastDrop = Double.NaN;
    private double lastRecoveryMs = Double.NaN;
    private int shots = 0;

    private double startP, startI, startD, startF, startVelocity, startHood;

    @Override
    public void runOpMode() {
        dash = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new Shooter(hardwareMap);
        hood = new Hood(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        gate = new Gate(hardwareMap);

        startP = RobotConstants.FLYWHEEL_P;
        startI = RobotConstants.FLYWHEEL_I;
        startD = RobotConstants.FLYWHEEL_D;
        startF = RobotConstants.FLYWHEEL_F;
        if (RobotConstants.FLYWHEEL_VELOCITY < MIN_TUNE_VELOCITY) {
            RobotConstants.FLYWHEEL_VELOCITY = DEFAULT_VELOCITY;
        }
        startVelocity = RobotConstants.FLYWHEEL_VELOCITY;
        startHood = RobotConstants.HOOD_UP;

        gate.setState(Gate.State.CLOSED);
        hood.setPosition(RobotConstants.HOOD_UP);

        dash.addLine("ShooterTuner ready. Intake, transfer and flywheel start at PLAY.");
        dash.addLine("Keep hands clear of the intake.");
        dash.update();

        waitForStart();
        if (isStopRequested()) return;

        stepTimer.reset();

        while (opModeIsActive()) {
            g1Prev.copy(g1);
            g1.copy(gamepad1);

            buttons();
            applyGains();
            applyVelocity();
            hood.setPosition(RobotConstants.HOOD_UP);
            applyFeed();
            measureShots();
            readyRumble();
            addTelemetry();
        }

        shooter.stop();
        intake.setState(Intake.State.IDLE);
        transfer.setState(Transfer.State.IDLE);
        gate.setState(Gate.State.CLOSED);
    }

    private void buttons() {
        if (pressed(g1.dpad_right, g1Prev.dpad_right)) {
            selected = Param.values()[(selected.ordinal() + 1) % Param.values().length];
        }
        if (pressed(g1.dpad_left, g1Prev.dpad_left)) {
            int n = Param.values().length;
            selected = Param.values()[(selected.ordinal() + n - 1) % n];
        }
        if (pressed(g1.dpad_up, g1Prev.dpad_up)) {
            change(+step());
        }
        if (pressed(g1.dpad_down, g1Prev.dpad_down)) {
            change(-step());
        }
        if (pressed(g1.right_bumper, g1Prev.right_bumper)) {
            stepScale[selected.ordinal()] = Math.min(100, stepScale[selected.ordinal()] * 10);
        }
        if (pressed(g1.left_bumper, g1Prev.left_bumper)) {
            stepScale[selected.ordinal()] = Math.max(0.001, stepScale[selected.ordinal()] / 10);
        }
        if (pressed(g1.x, g1Prev.x)) {
            flywheelOn = !flywheelOn;
            gamepad1.rumbleBlips(flywheelOn ? 2 : 1);
        }
        if (pressed(g1.b, g1Prev.b)) {
            stepTest = !stepTest;
            stepHigh = true;
            stepTimer.reset();
            gamepad1.rumbleBlips(stepTest ? 2 : 1);
        }
        if (pressed(g1.y, g1Prev.y)) {
            lastDrop = Double.NaN;
            lastRecoveryMs = Double.NaN;
            shots = 0;
            gamepad1.rumbleBlips(1);
        }
        if (pressed(g1.back, g1Prev.back)) {
            RobotConstants.FLYWHEEL_P = startP;
            RobotConstants.FLYWHEEL_I = startI;
            RobotConstants.FLYWHEEL_D = startD;
            RobotConstants.FLYWHEEL_F = startF;
            RobotConstants.FLYWHEEL_VELOCITY = startVelocity;
            RobotConstants.HOOD_UP = startHood;
            gamepad1.rumble(300);
        }
    }

    private double step() {
        return BASE_STEP[selected.ordinal()] * stepScale[selected.ordinal()];
    }

    private void change(double delta) {
        switch (selected) {
            case P:
                RobotConstants.FLYWHEEL_P = round(Math.max(0, RobotConstants.FLYWHEEL_P + delta));
                break;
            case D:
                RobotConstants.FLYWHEEL_D = round(Math.max(0, RobotConstants.FLYWHEEL_D + delta));
                break;
            case F:
                RobotConstants.FLYWHEEL_F = round(Math.max(0, RobotConstants.FLYWHEEL_F + delta));
                break;
            case VELOCITY:
                RobotConstants.FLYWHEEL_VELOCITY = round(Math.max(MIN_TUNE_VELOCITY, Math.min(MAX_VELOCITY, RobotConstants.FLYWHEEL_VELOCITY + delta)));
                break;
            case HOOD:
                RobotConstants.HOOD_UP = round(Math.max(0, Math.min(1, RobotConstants.HOOD_UP + delta)));
                break;
        }
    }

    private static double round(double v) {
        return Math.round(v * 10000.0) / 10000.0;
    }

    private void applyGains() {
        if (RobotConstants.FLYWHEEL_P != appliedP || RobotConstants.FLYWHEEL_I != appliedI
                || RobotConstants.FLYWHEEL_D != appliedD || RobotConstants.FLYWHEEL_F != appliedF) {
            shooter.refreshPIDF();
            appliedP = RobotConstants.FLYWHEEL_P;
            appliedI = RobotConstants.FLYWHEEL_I;
            appliedD = RobotConstants.FLYWHEEL_D;
            appliedF = RobotConstants.FLYWHEEL_F;
        }
    }

    private double commandedVelocity() {
        if (!flywheelOn) return 0;
        double base = Math.max(MIN_TUNE_VELOCITY, Math.min(MAX_VELOCITY, RobotConstants.FLYWHEEL_VELOCITY));
        if (stepTest) {
            if (stepTimer.milliseconds() >= STEP_TEST_PERIOD_MS) {
                stepHigh = !stepHigh;
                stepTimer.reset();
            }
            return stepHigh ? base : Math.max(MIN_TUNE_VELOCITY, base - STEP_TEST_DELTA);
        }
        return base;
    }

    private void applyVelocity() {
        double velocity = commandedVelocity();
        if (velocity != appliedVelocity) {
            boolean wasOff = !shooter.isOn();
            shooter.spin(velocity);
            if (wasOff && velocity > 0) {
                appliedP = RobotConstants.FLYWHEEL_P;
                appliedI = RobotConstants.FLYWHEEL_I;
                appliedD = RobotConstants.FLYWHEEL_D;
                appliedF = RobotConstants.FLYWHEEL_F;
            }
            appliedVelocity = velocity;
        }
    }

    private void applyFeed() {
        if (g1.left_trigger > TRIGGER_THRESHOLD) {
            intake.setState(Intake.State.OUTTAKE);
            transfer.setState(Transfer.State.REVERSE);
            gate.setState(Gate.State.CLOSED);
            return;
        }
        if (intake.getState() != Intake.State.INTAKE) intake.setState(Intake.State.INTAKE);
        if (transfer.getState() != Transfer.State.TRANSFER) transfer.setState(Transfer.State.TRANSFER);
        Gate.State wanted = g1.right_trigger > TRIGGER_THRESHOLD ? Gate.State.OPEN : Gate.State.CLOSED;
        if (gate.getState() != wanted) gate.setState(wanted);
    }

    private void measureShots() {
        if (stepTest || !shooter.isOn()) {
            inDip = false;
            return;
        }
        double target = shooter.getTargetVelocity();
        double average = (shooter.getVelocity1() + shooter.getVelocity2()) / 2.0;
        if (!inDip) {
            if (gate.getState() == Gate.State.OPEN && average < target - SHOT_DROP_TPS) {
                inDip = true;
                dipMin = average;
                shotTimer.reset();
            }
        } else {
            dipMin = Math.min(dipMin, average);
            if (Math.abs(average - target) <= RobotConstants.FLYWHEEL_READY_TOLERANCE) {
                inDip = false;
                shots++;
                lastDrop = target - dipMin;
                lastRecoveryMs = shotTimer.milliseconds();
            } else if (shotTimer.milliseconds() > 3000) {
                inDip = false;
            }
        }
    }

    private void readyRumble() {
        boolean ready = shooter.isReady();
        if (ready && !wasReady && !stepTest) {
            gamepad1.rumbleBlips(2);
        }
        wasReady = ready;
    }

    private void addTelemetry() {
        double v1 = shooter.getVelocity1();
        double v2 = shooter.getVelocity2();
        double target = shooter.getTargetVelocity();
        double average = (v1 + v2) / 2.0;

        dash.addData("target", target);
        dash.addData("velocity1", v1);
        dash.addData("velocity2", v2);
        dash.addData("average", average);
        dash.addData("error", target - average);

        dash.addLine();
        dash.addData(">> Editing", "%s   (step %s)", selected, fmt(step()));
        dash.addData(mark(Param.P) + "P", fmt(RobotConstants.FLYWHEEL_P));
        dash.addData(mark(Param.D) + "D", fmt(RobotConstants.FLYWHEEL_D));
        dash.addData(mark(Param.F) + "F", fmt(RobotConstants.FLYWHEEL_F));
        dash.addData(mark(Param.VELOCITY) + "Velocity (tps)", fmt(RobotConstants.FLYWHEEL_VELOCITY));
        dash.addData(mark(Param.HOOD) + "Hood", fmt(RobotConstants.HOOD_UP));
        dash.addData("   I (Dashboard only)", fmt(RobotConstants.FLYWHEEL_I));

        dash.addLine();
        dash.addData("Flywheel", !flywheelOn ? "OFF" : (stepTest ? "STEP TEST" : (shooter.isReady() ? "READY" : "spinning up")));
        dash.addData("Flywheel 1 / 2", "%.0f / %.0f  (target %.0f)", v1, v2, target);
        dash.addData("Gate", gate.getState());
        dash.addData("Feed", g1.left_trigger > TRIGGER_THRESHOLD ? "REVERSE (unjam)" : "intake + transfer ON");
        dash.addData("Shots measured", shots);
        dash.addData("Last shot drop (tps)", Double.isNaN(lastDrop) ? "-" : String.format(Locale.US, "%.0f", lastDrop));
        dash.addData("Last recovery (ms)", Double.isNaN(lastRecoveryMs) ? "-" : String.format(Locale.US, "%.0f", lastRecoveryMs));

        dash.addLine();
        dash.addLine("D-pad L/R pick | D-pad U/D change | RB/LB step x10 / /10");
        dash.addLine("X flywheel | B step test | RT shoot | LT unjam | Y reset shot stats | BACK undo all");
        dash.addLine(String.format(Locale.US,
                "Save to RobotConstants: FLYWHEEL_P = %s; FLYWHEEL_D = %s; FLYWHEEL_F = %s; FLYWHEEL_VELOCITY = %s; HOOD_UP = %s;",
                fmt(RobotConstants.FLYWHEEL_P), fmt(RobotConstants.FLYWHEEL_D), fmt(RobotConstants.FLYWHEEL_F),
                fmt(RobotConstants.FLYWHEEL_VELOCITY), fmt(RobotConstants.HOOD_UP)));
        dash.update();
    }

    private String mark(Param p) {
        return p == selected ? "> " : "   ";
    }

    private static String fmt(double v) {
        String s = String.format(Locale.US, "%.4f", v);
        s = s.replaceAll("0+$", "");
        return s.endsWith(".") ? s + "0" : s;
    }

    private static boolean pressed(boolean now, boolean before) {
        return now && !before;
    }
}
