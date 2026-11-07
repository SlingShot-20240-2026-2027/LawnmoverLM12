package org.firstinspires.ftc.teamcode.BareAuto.sdk;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Shooter {

    public static String FLYWHEEL1_NAME = "flywheel1";
    public static String FLYWHEEL2_NAME = "flywheel2";
    public static String HOOD_SERVO_NAME = "hood";
    public static boolean FLYWHEEL1_REVERSED = true;
    public static boolean FLYWHEEL2_REVERSED = true;

    public static double TARGET_VELOCITY = 1150;
    public static double MAX_VELOCITY = 2800;
    public static double HOOD_POSITION = 0.4;
    public static double HOOD_MIN = 0.0;
    public static double HOOD_MAX = 1.0;

    public static double kP = 1.0;
    public static double kI = 0.0;
    public static double kD = 0.01;
    public static double kF = 17.6;

    public static boolean VOLTAGE_COMPENSATION = true;
    public static double NOMINAL_VOLTAGE = 12.0;
    public static long VOLTAGE_PERIOD_MS = 200;

    public static double READY_TOLERANCE = 30;
    public static long READY_HOLD_MS = 80;
    public static long SPINUP_TIMEOUT_MS = 2500;

    public enum Mode {ON, IDLE}

    private final DcMotorEx flywheel1;
    private final DcMotorEx flywheel2;
    private final DcMotorEx[] motors;
    private final Servo hood;
    private final VoltageSensor voltageSensor;

    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();

    private Mode mode = Mode.IDLE;
    private double velocityOverride = Double.NaN;
    private double hoodOverride = Double.NaN;

    private boolean velocityModeActive = false;
    private double commandedVelocity = Double.NaN;
    private double appliedP = Double.NaN, appliedI = Double.NaN, appliedD = Double.NaN, appliedF = Double.NaN;

    private double velocity = 0, velocity1 = 0, velocity2 = 0, targetVelocity = 0, error = 0;
    private double lastHood = Double.NaN;
    private double voltage;
    private boolean ready = false;

    public Shooter(HardwareMap hardwareMap) {
        flywheel1 = hardwareMap.get(DcMotorEx.class, FLYWHEEL1_NAME);
        flywheel2 = hardwareMap.get(DcMotorEx.class, FLYWHEEL2_NAME);
        motors = new DcMotorEx[]{flywheel1, flywheel2};
        hood = hardwareMap.get(Servo.class, HOOD_SERVO_NAME);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        flywheel1.setDirection(FLYWHEEL1_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        flywheel2.setDirection(FLYWHEEL2_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        for (DcMotorEx m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setPower(0);
        }
        voltage = voltageSensor.getVoltage();
        writeHood(HOOD_POSITION);
    }

    public Action on() {
        return spinUp(Double.NaN, Double.NaN);
    }

    public Action on(double velocity, double hoodPosition) {
        return spinUp(velocity, hoodPosition);
    }

    public Action idle() {
        return Actions.instant(this::setIdle);
    }

    public Action waitUntilReady(long timeoutMs) {
        return Actions.waitUntil(this::isReady, timeoutMs);
    }

    private Action spinUp(double velocity, double hoodPosition) {
        ElapsedTime timer = new ElapsedTime();
        return new Action() {
            public void start() {
                setOn(velocity, hoodPosition);
                timer.reset();
            }

            public boolean update() {
                return ready || timer.milliseconds() >= SPINUP_TIMEOUT_MS;
            }

            public void stop() {
            }
        };
    }

    public void setOn() {
        setOn(Double.NaN, Double.NaN);
    }

    public void setOn(double velocity, double hoodPosition) {
        velocityOverride = velocity;
        hoodOverride = hoodPosition;
        mode = Mode.ON;
        resetReady();
    }

    public void setIdle() {
        if (mode != Mode.IDLE) {
            mode = Mode.IDLE;
            resetReady();
        }
    }

    public void update() {
        if (voltageTimer.milliseconds() >= VOLTAGE_PERIOD_MS) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        velocity1 = flywheel1.getVelocity();
        velocity2 = flywheel2.getVelocity();
        velocity = (velocity1 + velocity2) / 2.0;

        double wanted = mode == Mode.ON ? (Double.isNaN(velocityOverride) ? TARGET_VELOCITY : velocityOverride) : 0.0;
        targetVelocity = Range.clip(wanted, 0.0, MAX_VELOCITY);
        error = targetVelocity - velocity;

        if (targetVelocity > 0) {
            runVelocity(targetVelocity);
        } else {
            coast();
        }

        writeHood(Double.isNaN(hoodOverride) ? HOOD_POSITION : hoodOverride);
        updateReady();
    }

    public void stop() {
        mode = Mode.IDLE;
        targetVelocity = 0;
        ready = false;
        coast();
    }

    private void runVelocity(double ticksPerSec) {
        if (!velocityModeActive) {
            for (DcMotorEx m : motors) m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            velocityModeActive = true;
            appliedP = appliedI = appliedD = appliedF = Double.NaN;
            commandedVelocity = Double.NaN;
        }
        applyCoefficientsIfChanged();
        if (Double.isNaN(commandedVelocity) || Math.abs(ticksPerSec - commandedVelocity) > 0.5) {
            for (DcMotorEx m : motors) m.setVelocity(ticksPerSec);
            commandedVelocity = ticksPerSec;
        }
    }

    private void coast() {
        if (velocityModeActive || Double.isNaN(commandedVelocity)) {
            for (DcMotorEx m : motors) {
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setPower(0);
            }
            velocityModeActive = false;
            commandedVelocity = 0;
        }
    }

    private void applyCoefficientsIfChanged() {
        double scale = VOLTAGE_COMPENSATION ? NOMINAL_VOLTAGE / Math.max(voltage, 6.0) : 1.0;
        double f = kF * scale;
        boolean changed = kP != appliedP || kI != appliedI || kD != appliedD
                || Double.isNaN(appliedF) || Math.abs(f - appliedF) > Math.abs(appliedF) * 0.01;
        if (!changed) return;
        for (DcMotorEx m : motors) m.setVelocityPIDFCoefficients(kP, kI, kD, f);
        appliedP = kP;
        appliedI = kI;
        appliedD = kD;
        appliedF = f;
    }

    private void resetReady() {
        readyTimer.reset();
        ready = false;
    }

    private void updateReady() {
        boolean inBand = targetVelocity > 0 && Math.abs(error) <= READY_TOLERANCE;
        if (!inBand) {
            readyTimer.reset();
            ready = false;
        } else {
            ready = readyTimer.milliseconds() >= READY_HOLD_MS;
        }
    }

    private void writeHood(double position) {
        double clipped = Range.clip(position, HOOD_MIN, HOOD_MAX);
        if (Double.isNaN(lastHood) || Math.abs(clipped - lastHood) > 0.0005) {
            hood.setPosition(clipped);
            lastHood = clipped;
        }
    }

    public boolean isReady() {
        return ready;
    }

    public Mode getMode() {
        return mode;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getVelocity1() {
        return velocity1;
    }

    public double getVelocity2() {
        return velocity2;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getError() {
        return error;
    }

    public double getVoltage() {
        return voltage;
    }

    public double getHood() {
        return lastHood;
    }

    public double getAppliedF() {
        return Double.isNaN(appliedF) ? 0 : appliedF;
    }

    public double getCurrentAmps() {
        return flywheel1.getCurrent(CurrentUnit.AMPS) + flywheel2.getCurrent(CurrentUnit.AMPS);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter mode", mode);
        telemetry.addData("Shooter ready", ready);
        telemetry.addData("Shooter tps (target / avg / 1 / 2)", "%.0f / %.0f / %.0f / %.0f",
                targetVelocity, velocity, velocity1, velocity2);
        telemetry.addData("Shooter PIDF", "%.3f / %.3f / %.3f / %.3f", kP, kI, kD, getAppliedF());
        telemetry.addData("Shooter hood", "%.3f", lastHood);
        telemetry.addData("Battery voltage", "%.2f", voltage);
    }
}
