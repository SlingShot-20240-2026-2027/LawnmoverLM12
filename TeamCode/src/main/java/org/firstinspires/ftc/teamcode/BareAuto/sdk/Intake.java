package org.firstinspires.ftc.teamcode.BareAuto.sdk;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class Intake {

    public static String MOTOR_NAME = "intake";
    public static boolean REVERSED = true;
    public static double INTAKE_POWER = 1.0;

    public enum State {INTAKE, OUTTAKE, IDLE}

    private final DcMotorEx motor;
    private State state = State.IDLE;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
    }

    public Action in() {
        return Actions.instant(() -> setState(State.INTAKE));
    }

    public Action out() {
        return Actions.instant(() -> setState(State.OUTTAKE));
    }

    public Action idle() {
        return Actions.instant(() -> setState(State.IDLE));
    }

    public void setState(State newState) {
        state = newState;
        double power = Range.clip(Math.abs(INTAKE_POWER), 0.0, 1.0);
        switch (newState) {
            case INTAKE:
                motor.setPower(power);
                break;
            case OUTTAKE:
                motor.setPower(-power);
                break;
            case IDLE:
            default:
                motor.setPower(0);
                break;
        }
    }

    public State getState() {
        return state;
    }
}
