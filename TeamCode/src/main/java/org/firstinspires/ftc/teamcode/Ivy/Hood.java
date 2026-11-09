package org.firstinspires.ftc.teamcode.Ivy;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Hood {

    public enum State {UP, DOWN};
    private final Servo hood;
    private State state = State.DOWN;
    public Hood(HardwareMap hardwareMap){
        hood = hardwareMap.get(Servo.class, RobotConstants.HOOD_NAME);
    }

    public void setState (State newState){
        state = newState;
        switch (newState){
            case UP:
                hood.setPosition(RobotConstants.HOOD_UP);
                break;
            case DOWN:
                hood.setPosition(RobotConstants.HOOD_DOWN);
                break;
        }
    }

    public State getState() {
        return state;
    }

    public Command up(){return instant(() -> setState(State.UP)).requiring(this);}

    public Command down(){return instant(() -> setState(State.DOWN)).requiring(this);}
}
