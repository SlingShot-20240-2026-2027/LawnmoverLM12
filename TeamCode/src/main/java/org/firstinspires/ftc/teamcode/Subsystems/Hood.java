package org.firstinspires.ftc.teamcode.Subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {

    public enum State {UP, DOWN};
    private final Servo gate;
    private State state = State.DOWN;
    public Hood(HardwareMap hardwareMap){
        gate = hardwareMap.get(Servo.class, "hood");
    }

    public void setState (State newState){
        state = newState;
        switch (newState){
            case UP:
                // Change servo positions once bot is done
                gate.setPosition(0.43);
                break;
            case DOWN:
                gate.setPosition(0.0);
                break;
        }
    }

    public Command up(){return instant(() -> setState(State.UP)).requiring(this);}

    public Command down(){return instant(() -> setState(State.DOWN)).requiring(this);}
}
