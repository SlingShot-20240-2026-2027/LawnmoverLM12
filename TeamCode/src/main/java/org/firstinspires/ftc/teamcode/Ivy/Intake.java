package org.firstinspires.ftc.teamcode.Ivy;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

public class Intake {

    public enum State {INTAKE, OUTTAKE, IDLE}

    private final DcMotorEx intake;
    private State state = State.IDLE;

    public Intake(HardwareMap hardwareMap) {
        this(hardwareMap, RobotConstants.INTAKE_NAME);
    }

    public Intake(HardwareMap hardwareMap, String motorName) {
        intake = hardwareMap.get(DcMotorEx.class, motorName);
        intake.setDirection(RobotConstants.INTAKE_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0);
    }

    public void setState(State newState) {
        state = newState;
        applyPower();
    }

    public State getState() {
        return state;
    }

    public void update() {
    }

    private void applyPower() {
        switch (state) {
            case INTAKE:
                intake.setPower(Range.clip(Math.abs(RobotConstants.INTAKE_POWER), 0.0, 1.0));
                break;
            case OUTTAKE:
                intake.setPower(-Range.clip(Math.abs(RobotConstants.OUTTAKE_POWER), 0.0, 1.0));
                break;
            case IDLE:
            default:
                intake.setPower(0);
                break;
        }
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake State", state);
    }

    public Command in() {
        return instant(() -> setState(State.INTAKE)).requiring(this);
    }

    public Command out() {
        return instant(() -> setState(State.OUTTAKE)).requiring(this);
    }

    public Command idle() {
        return instant(() -> setState(State.IDLE)).requiring(this);
    }
}
