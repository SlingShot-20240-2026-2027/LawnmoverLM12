package org.firstinspires.ftc.teamcode.Auton.Ivy;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Auton.RobotConstants;

public class Transfer {

    public enum State {TRANSFER, REVERSE, IDLE}
    private final DcMotorEx transfer;
    private State state = State.IDLE;

    public Transfer(HardwareMap hardwareMap) {
        this(hardwareMap, RobotConstants.TRANSFER_NAME);
    }

    public Transfer(HardwareMap hardwareMap, String name) {
        transfer = hardwareMap.get(DcMotorEx.class, name);
        transfer.setDirection(RobotConstants.TRANSFER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setPower(0);
    }

    public void setState(State newState) {
        state = newState;
        switch (newState) {
            case TRANSFER:
                transfer.setPower(Range.clip(Math.abs(RobotConstants.TRANSFER_POWER), 0.0, 1.0));
                break;
            case REVERSE:
                transfer.setPower(-Range.clip(Math.abs(RobotConstants.TRANSFER_REVERSE_POWER), 0.0, 1.0));
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

    public Command reverse() {
        return instant(() -> setState(State.REVERSE)).requiring(this);
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
