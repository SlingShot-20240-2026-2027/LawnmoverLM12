package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TransferAuto {

    public enum State {TRANSFER, IDLE}
    private final DcMotorEx transfer;
    private State state = State.IDLE;

    public TransferAuto(HardwareMap hardwareMap) {
        this(hardwareMap, "transfer");
    }

    public TransferAuto(HardwareMap hardwareMap, String name) {
        transfer = hardwareMap.get(DcMotorEx.class, name);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setState(State newState) {
        state = newState;
        switch (newState) {
            case TRANSFER:
                transfer.setPower(1.0);
                break;
            case IDLE:
            default:
                transfer.setPower(0);
                break;
        }
    }

    public State getState() {
        return state;
    }

    public Command transfer() {
        return instant(() -> setState(State.TRANSFER)).requiring(this);
    }

    public Command idle() {
        return instant(() -> setState(State.IDLE)).requiring(this);
    }

    public Command transfer(long timeMs) {
        ElapsedTime timer = new ElapsedTime();
        return Command.build()
                .setStart(() -> {
                    timer.reset();
                    setState(State.TRANSFER);
                })
                .setDone(() -> timer.milliseconds() >= timeMs)
                .setEnd(endCondition -> setState(State.IDLE))
                .requiring(this);
    }
}
