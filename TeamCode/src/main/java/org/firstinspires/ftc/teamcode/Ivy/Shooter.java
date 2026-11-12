package org.firstinspires.ftc.teamcode.Ivy;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

public class Shooter {

    private final DcMotorEx flywheel1;
    private final DcMotorEx flywheel2;

    private boolean on = false;
    private double targetVelocity = 0;

    public Shooter(HardwareMap hardwareMap) {
        flywheel1 = hardwareMap.get(DcMotorEx.class, RobotConstants.FLYWHEEL1_NAME);
        flywheel2 = hardwareMap.get(DcMotorEx.class, RobotConstants.FLYWHEEL2_NAME);

        flywheel1.setDirection(RobotConstants.FLYWHEEL1_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        flywheel2.setDirection(RobotConstants.FLYWHEEL2_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        coast();
    }

    public Command on() {
        return instant(() -> spin(RobotConstants.FLYWHEEL_VELOCITY)).requiring(this);
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
            flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            applyPIDF();
            on = true;
        }
        targetVelocity = velocity;
        flywheel1.setVelocity(velocity);
        flywheel2.setVelocity(velocity);
    }

    public void refreshPIDF() {
        if (on) {
            applyPIDF();
        }
    }

    private void applyPIDF() {
        PIDFCoefficients pidf = new PIDFCoefficients(
                RobotConstants.FLYWHEEL_P, RobotConstants.FLYWHEEL_I,
                RobotConstants.FLYWHEEL_D, RobotConstants.FLYWHEEL_F);
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
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
        double tolerance = RobotConstants.FLYWHEEL_READY_TOLERANCE;
        return on
                && Math.abs(flywheel1.getVelocity() - targetVelocity) <= tolerance
                && Math.abs(flywheel2.getVelocity() - targetVelocity) <= tolerance;
    }

    public boolean isOn() {
        return on;
    }

    public double getVelocity1() {
        return flywheel1.getVelocity();
    }

    public double getVelocity2() {
        return flywheel2.getVelocity();
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
