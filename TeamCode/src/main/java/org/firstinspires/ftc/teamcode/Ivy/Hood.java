package org.firstinspires.ftc.teamcode.Ivy;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Hood {

    public enum State {UP, DOWN};
    private final Servo hood;
    private State state = State.DOWN;
    private double lastPosition = Double.NaN;
    public Hood(HardwareMap hardwareMap){
        hood = hardwareMap.get(Servo.class, RobotConstants.HOOD_NAME);
    }

    public void setState (State newState){
        state = newState;
        switch (newState){
            case UP:
                write(RobotConstants.HOOD_UP);
                break;
            case DOWN:
                write(RobotConstants.HOOD_DOWN);
                break;
        }
    }

    public void setPosition(double position) {
        double clipped = Range.clip(position, 0.0, 1.0);
        if (Double.isNaN(lastPosition) || Math.abs(clipped - lastPosition) > 0.001) {
            write(clipped);
        }
    }

    public double getPosition() {
        return lastPosition;
    }

    private void write(double position) {
        hood.setPosition(position);
        lastPosition = position;
    }

    public State getState() {
        return state;
    }

    public Command up(){return instant(() -> setState(State.UP)).requiring(this);}

    public Command down(){return instant(() -> setState(State.DOWN)).requiring(this);}
}
