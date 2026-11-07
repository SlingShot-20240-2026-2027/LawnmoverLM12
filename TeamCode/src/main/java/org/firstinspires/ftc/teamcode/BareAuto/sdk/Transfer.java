package org.firstinspires.ftc.teamcode.BareAuto.sdk;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class Transfer {

    public static String MOTOR_NAME = "transfer";
    public static String GATE_NAME = "gate";
    public static boolean REVERSED = true;
    public static double FEED_POWER = 1.0;
    public static long FEED_MS = 1000;
    public static double GATE_CLOSED = 0.66;
    public static double GATE_OPEN = 0.4;
    public static long GATE_OPEN_MS = 150;

    public enum State {FEED, REVERSE, IDLE}

    private final DcMotorEx motor;
    private final Servo gate;
    private State state = State.IDLE;

    public Transfer(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
        gate = hardwareMap.get(Servo.class, GATE_NAME);
        gate.setPosition(GATE_CLOSED);
    }

    public Action transfer() {
        return transfer(FEED_MS);
    }

    public Action transfer(long timeMs) {
        ElapsedTime timer = new ElapsedTime();
        return new Action() {
            public void start() {
                timer.reset();
                gate.setPosition(GATE_OPEN);
                setState(State.IDLE);
            }

            public boolean update() {
                double t = timer.milliseconds();
                if (t >= GATE_OPEN_MS + timeMs) {
                    setState(State.IDLE);
                    gate.setPosition(GATE_CLOSED);
                    return true;
                }
                if (t >= GATE_OPEN_MS && state != State.FEED) {
                    setState(State.FEED);
                }
                return false;
            }

            public void stop() {
                setState(State.IDLE);
                gate.setPosition(GATE_CLOSED);
            }
        };
    }

    public Action idle() {
        return Actions.instant(() -> {
            setState(State.IDLE);
            gate.setPosition(GATE_CLOSED);
        });
    }

    public void setState(State newState) {
        state = newState;
        double power = Range.clip(Math.abs(FEED_POWER), 0.0, 1.0);
        switch (newState) {
            case FEED:
                motor.setPower(power);
                break;
            case REVERSE:
                motor.setPower(-power);
                break;
            case IDLE:
            default:
                motor.setPower(0);
                break;
        }
    }

    public void closeGate() {
        gate.setPosition(GATE_CLOSED);
    }

    public State getState() {
        return state;
    }
}
