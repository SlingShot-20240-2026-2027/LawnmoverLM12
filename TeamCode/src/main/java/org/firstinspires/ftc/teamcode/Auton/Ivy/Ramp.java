package org.firstinspires.ftc.teamcode.Auton.Ivy;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Auton.RobotConstants;

public class Ramp {
    public enum State {UP, DEPLOY};
    private final Servo ramp;
    private State state = State.UP;
    public Ramp(HardwareMap hardwareMap){ramp = hardwareMap.get(Servo.class, RobotConstants.DEPLOY_NAME);}

    public void setState (State newState){
        state = newState;
        switch (newState){
            case UP:
                ramp.setPosition(RobotConstants.DEPLOY_UP);
                break;
            case DEPLOY:
                ramp.setPosition(RobotConstants.DEPLOY_DOWN);
                break;
        }
    }

    public State getState() {
        return state;
    }

    public Command up(){return instant(() -> setState(State.UP)).requiring(this);}

    public Command deploy(){return instant(() -> setState(State.DEPLOY)).requiring(this);}
}
