package org.firstinspires.ftc.teamcode.Subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Shooter {

    public static String FLYWHEEL1_NAME = "flywheel1";
    public static String FLYWHEEL2_NAME = "flywheel2";

    public static double FLYWHEEL_VELOCITY = 1350;

    public static double FLYWHEEL_P = 0.0;
    public static double FLYWHEEL_I = 0.0;
    public static double FLYWHEEL_D = 0.0;
    public static double FLYWHEEL_F = 14.0;

    public static double READY_TOLERANCE = 30;

    private final DcMotorEx flywheel1;
    private final DcMotorEx flywheel2;

    private boolean on = false;
    private double targetVelocity = 0;

    public Shooter(HardwareMap hardwareMap) {
        flywheel1 = hardwareMap.get(DcMotorEx.class, FLYWHEEL1_NAME);
        flywheel2 = hardwareMap.get(DcMotorEx.class, FLYWHEEL2_NAME);

        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        coast();
    }

    public Command on() {
        return instant(() -> spin(FLYWHEEL_VELOCITY)).requiring(this);
    }

    public Command on(double velocity) {
        return instant(() -> spin(velocity)).requiring(this);
    }

    public Command idle() {
        return instant(this::stop).requiring(this);
    }

    public Command waitUntilReady(long timeoutMs) {
        ElapsedTime timer = new ElapsedTime();
        return Command.build()
                .setStart(timer::reset)
                .setDone(() -> isReady() || timer.milliseconds() >= timeoutMs);
    }

    public Command periodic() {
        return Command.build()
                .setEnd(endCondition -> stop());
    }

    public void spin(double velocity) {
        if (velocity <= 0) {
            stop();
            return;
        }
        if (!on) {
            PIDFCoefficients pidf = new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);
            flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            on = true;
        }
        targetVelocity = velocity;
        flywheel1.setVelocity(velocity);
        flywheel2.setVelocity(velocity);
    }

    public void stop() {
        on = false;
        targetVelocity = 0;
        coast();
    }

    private void coast() {
        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setPower(0);
        flywheel2.setPower(0);
    }

    public boolean isReady() {
        return on
                && Math.abs(flywheel1.getVelocity() - targetVelocity) <= READY_TOLERANCE
                && Math.abs(flywheel2.getVelocity() - targetVelocity) <= READY_TOLERANCE;
    }

    public boolean isOn() {
        return on;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter", on ? "ON" : "IDLE");
        telemetry.addData("Shooter ready", isReady());
        telemetry.addData("Flywheel target", "%.0f", targetVelocity);
        telemetry.addData("Flywheel 1 vel", "%.0f", flywheel1.getVelocity());
        telemetry.addData("Flywheel 2 vel", "%.0f", flywheel2.getVelocity());
    }
}
