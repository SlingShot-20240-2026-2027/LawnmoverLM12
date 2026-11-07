package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GateAuto {

    public enum State {CLOSED, OPEN};
    private final Servo gate;
    private State state = State.CLOSED;
    public GateAuto(HardwareMap hardwareMap){
        gate = hardwareMap.get(Servo.class, "gate");
    }

    public void setState (State newState){
        state = newState;
        switch (newState){
            case CLOSED:
                // Change servo positions once bot is done
                gate.setPosition(0.66);
                break;
            case OPEN:
                gate.setPosition(0.4);
                break;
        }
    }

    public Command cl(){return instant(() -> setState(State.CLOSED)).requiring(this);}

    public Command op(){return instant(() -> setState(State.OPEN)).requiring(this);}
}
