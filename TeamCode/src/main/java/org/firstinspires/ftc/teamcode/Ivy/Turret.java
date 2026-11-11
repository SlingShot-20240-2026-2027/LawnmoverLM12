package org.firstinspires.ftc.teamcode.Ivy;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TurretConstants;

public class Turret {

    public enum Target {TARGET_1, TARGET_2}

    public enum Mode {AUTO_AIM, MANUAL}

    private final Servo servo1;
    private final Servo servo2;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private Mode mode = Mode.AUTO_AIM;
    private Target target = Target.TARGET_1;
    private double trimDeg = 0;

    private double desiredDeg = 0;
    private double commandedDeg = 0;
    private double estimatedDeg = 0;
    private double distanceIn = Double.NaN;
    private boolean outOfRange = false;
    private boolean servoLimited = false;

    private boolean havePrev = false;
    private double prevX, prevY, prevH;
    private double vx, vy, omega;

    public Turret(HardwareMap hardwareMap) {
        servo1 = hardwareMap.get(Servo.class, TurretConstants.SERVO1_NAME);
        servo2 = hardwareMap.get(Servo.class, TurretConstants.SERVO2_NAME);
        servo1.setDirection(TurretConstants.SERVO1_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo2.setDirection(TurretConstants.SERVO2_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        command(0);
        estimatedDeg = commandedDeg;
        loopTimer.reset();
    }

    public void update(double robotX, double robotY, double robotHeadingRad) {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        updateVelocity(robotX, robotY, robotHeadingRad, dt);

        if (mode == Mode.AUTO_AIM) {
            double look = Math.max(0, TurretConstants.LOOKAHEAD_SEC);
            double px = robotX + vx * look;
            double py = robotY + vy * look;
            double ph = robotHeadingRad + omega * look;

            double c = Math.cos(ph);
            double s = Math.sin(ph);
            double tx = px + TurretConstants.TURRET_OFFSET_FORWARD_IN * c - TurretConstants.TURRET_OFFSET_LEFT_IN * s;
            double ty = py + TurretConstants.TURRET_OFFSET_FORWARD_IN * s + TurretConstants.TURRET_OFFSET_LEFT_IN * c;

            double dx = targetX() - tx;
            double dy = targetY() - ty;
            distanceIn = Math.hypot(dx, dy);

            if (distanceIn >= TurretConstants.MIN_TARGET_DISTANCE_IN) {
                double relative = Math.toDegrees(wrapRad(Math.atan2(dy, dx) - ph));
                desiredDeg = relative + trimDeg;
                command(chooseInRange(desiredDeg));
            }
        } else {
            distanceIn = Double.NaN;
            desiredDeg = TurretConstants.MANUAL_ANGLE_DEG + trimDeg;
            command(chooseInRange(desiredDeg));
        }

        double maxStep = Math.max(0, TurretConstants.TURRET_DEG_PER_SEC) * dt;
        double diff = commandedDeg - estimatedDeg;
        estimatedDeg += Range.clip(diff, -maxStep, maxStep);
    }

    private void updateVelocity(double x, double y, double h, double dt) {
        if (!havePrev || dt <= 0 || dt > 0.5) {
            havePrev = true;
            vx = vy = omega = 0;
        } else {
            double alpha = Range.clip(TurretConstants.VELOCITY_FILTER, 0.0, 1.0);
            vx = alpha * ((x - prevX) / dt) + (1 - alpha) * vx;
            vy = alpha * ((y - prevY) / dt) + (1 - alpha) * vy;
            omega = alpha * (wrapRad(h - prevH) / dt) + (1 - alpha) * omega;
        }
        prevX = x;
        prevY = y;
        prevH = h;
    }

    private double chooseInRange(double angleDeg) {
        double min = Math.min(TurretConstants.TURRET_MIN_DEG, TurretConstants.TURRET_MAX_DEG);
        double max = Math.max(TurretConstants.TURRET_MIN_DEG, TurretConstants.TURRET_MAX_DEG);

        double best = Double.NaN;
        for (int k = -2; k <= 2; k++) {
            double candidate = angleDeg + 360.0 * k;
            if (candidate >= min && candidate <= max) {
                if (Double.isNaN(best) || Math.abs(candidate - commandedDeg) < Math.abs(best - commandedDeg)) {
                    best = candidate;
                }
            }
        }
        if (!Double.isNaN(best)) {
            outOfRange = false;
            return best;
        }
        outOfRange = true;
        double toMin = Math.abs(Math.toDegrees(wrapRad(Math.toRadians(angleDeg - min))));
        double toMax = Math.abs(Math.toDegrees(wrapRad(Math.toRadians(angleDeg - max))));
        return toMin <= toMax ? min : max;
    }

    private void command(double angleDeg) {
        double unitsPerDeg = 1.0 / Math.max(1e-6, TurretConstants.DEGREES_PER_SERVO_UNIT);
        double lo = Math.min(TurretConstants.SERVO_MIN_POSITION, TurretConstants.SERVO_MAX_POSITION);
        double hi = Math.max(TurretConstants.SERVO_MIN_POSITION, TurretConstants.SERVO_MAX_POSITION);

        double p1 = TurretConstants.SERVO_CENTER + angleDeg * unitsPerDeg;
        double p2 = p1 + TurretConstants.SERVO2_TRIM;
        double c1 = Range.clip(p1, lo, hi);
        double c2 = Range.clip(p2, lo, hi);
        servoLimited = c1 != p1 || c2 != p2;

        servo1.setPosition(c1);
        servo2.setPosition(c2);
        commandedDeg = (c1 - TurretConstants.SERVO_CENTER) / unitsPerDeg;
    }

    private static double wrapRad(double a) {
        a = a % (2 * Math.PI);
        if (a > Math.PI) a -= 2 * Math.PI;
        if (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private double targetX() {
        return target == Target.TARGET_1 ? TurretConstants.TARGET1_X : TurretConstants.TARGET2_X;
    }

    private double targetY() {
        return target == Target.TARGET_1 ? TurretConstants.TARGET1_Y : TurretConstants.TARGET2_Y;
    }

    public void setMode(Mode newMode) {
        mode = newMode;
    }

    public Mode getMode() {
        return mode;
    }

    public void toggleMode() {
        mode = mode == Mode.AUTO_AIM ? Mode.MANUAL : Mode.AUTO_AIM;
    }

    public void setTarget(Target newTarget) {
        target = newTarget;
    }

    public Target getTarget() {
        return target;
    }

    public void toggleTarget() {
        target = target == Target.TARGET_1 ? Target.TARGET_2 : Target.TARGET_1;
    }

    public void addTrim(double deltaDeg) {
        double limit = Math.abs(TurretConstants.MAX_AIM_TRIM_DEG);
        trimDeg = Range.clip(trimDeg + deltaDeg, -limit, limit);
    }

    public void resetTrim() {
        trimDeg = 0;
    }

    public double getTrimDeg() {
        return trimDeg;
    }

    public boolean isOnTarget() {
        return !outOfRange && !servoLimited
                && Math.abs(commandedDeg - estimatedDeg) <= TurretConstants.ON_TARGET_TOLERANCE_DEG
                && (mode == Mode.MANUAL || !Double.isNaN(distanceIn));
    }

    public boolean isOutOfRange() {
        return outOfRange || servoLimited;
    }

    public double getCommandedDeg() {
        return commandedDeg;
    }

    public double getDistanceIn() {
        return distanceIn;
    }

    public Command autoAim() {
        return instant(() -> setMode(Mode.AUTO_AIM)).requiring(this);
    }

    public Command manual() {
        return instant(() -> setMode(Mode.MANUAL)).requiring(this);
    }

    public Command aimAt(Target newTarget) {
        return instant(() -> {
            setTarget(newTarget);
            setMode(Mode.AUTO_AIM);
        }).requiring(this);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret", "%s  %s  trim %+.0f", mode, target, trimDeg);
        telemetry.addData("Turret angle (want / cmd / est)", "%.1f / %.1f / %.1f",
                desiredDeg, commandedDeg, estimatedDeg);
        telemetry.addData("Turret status", isOutOfRange() ? "OUT OF RANGE" : (isOnTarget() ? "ON TARGET" : "moving"));
        if (!Double.isNaN(distanceIn)) {
            telemetry.addData("Target distance (in)", "%.1f", distanceIn);
        }
    }
}
