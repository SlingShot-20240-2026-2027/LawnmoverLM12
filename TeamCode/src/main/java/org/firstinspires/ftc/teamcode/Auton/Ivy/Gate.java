package org.firstinspires.ftc.teamcode.Auton.Ivy;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Auton.RobotConstants;

public class Gate {

    public enum State {CLOSED, OPEN};
    private final Servo gate;
    private State state = State.CLOSED;
    public Gate(HardwareMap hardwareMap){
        gate = hardwareMap.get(Servo.class, RobotConstants.GATE_NAME);
    }

    public void setState (State newState){
        state = newState;
        switch (newState){
            case CLOSED:
                gate.setPosition(RobotConstants.GATE_CLOSED);
                break;
            case OPEN:
                gate.setPosition(RobotConstants.GATE_OPEN);
                break;
        }
    }

    public State getState() {
        return state;
    }

    public Command cl(){return instant(() -> setState(State.CLOSED)).requiring(this);}

    public Command op(){return instant(() -> setState(State.OPEN)).requiring(this);}
}
