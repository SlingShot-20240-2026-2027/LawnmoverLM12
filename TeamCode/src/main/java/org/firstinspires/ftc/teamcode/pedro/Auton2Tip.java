package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Ivy.Hood;
import org.firstinspires.ftc.teamcode.Ivy.Intake;
import org.firstinspires.ftc.teamcode.Ivy.Transfer;
import org.firstinspires.ftc.teamcode.Ivy.Shooter;
import org.firstinspires.ftc.teamcode.Ivy.Gate;
import org.firstinspires.ftc.teamcode.Ivy.Ramp;


@Autonomous(name = "", group = "Autonomous")
public class Auton2Tip extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.8211, 9.0733, 90);
    private final Pose move1 = poseFactory.of(22.3692, 46.5544, 180);
    private final Pose intake1 = poseFactory.of(15.1947, 46.475, 180);
    private final Pose move2 = poseFactory.of(22.37, 46.55, 180);
    private final Pose move3Start = poseFactory.of(22.37, 46.55, 180);
    private final Pose move3 = poseFactory.of(22.5708, 111.2059, 270);
    private final Pose shooter2 = poseFactory.of(60.2298, 132.1451, 270);
    private final Pose shooter2Control1 = poseFactory.of(56.5692, 110.8359, 0);
    private final Pose move4 = poseFactory.of(46.2956, 114.7827, 270);
    private final Pose move4Control1 = poseFactory.of(60.2571, 112.6717, 0);
    private final Pose intake2 = poseFactory.of(46.3288, 125.4451, 89.8216);
    private final Pose move5 = poseFactory.of(46.3839, 114.5555, 90);
    private final Pose shooter3 = poseFactory.of(60.1459, 131.6372, -90.7277);
    private final Pose shooter3Control1 = poseFactory.of(59.9234, 114.1114, 0);
    private final Pose park = poseFactory.of(12.6335, 97.9859, 270);
    private final Pose parkControl1 = poseFactory.of(50.6917, 100.9653, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, move1())
//                follow(follower, intake1()),
//                follow(follower, move2()),
//                follow(follower, move3()),
//                follow(follower, shooter2()),
//                follow(follower, move4()),
//                follow(follower, intake2()),
//                follow(follower, move5()),
//                follow(follower, shooter3()),
//                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path move1() {
        return Paths.line(start, move1).linear(start, move1);
    }

    public Path intake1() {
        return Paths.line(move1, intake1).constant(intake1);
    }

    public Path move2() {
        return Paths.line(intake1, move2).constant(move2);
    }

    public Path move3() {
        return Paths.line(move3Start, move3).linear(move3Start, move3);
    }

    public Path shooter2() {
        return Paths.curve(move3, shooter2Control1, shooter2).constant(shooter2);
    }

    public Path move4() {
        return Paths.curve(shooter2, move4Control1, move4).constant(move4);
    }

    public Path intake2() {
        return Paths.line(move4, intake2).tangent();
    }

    public Path move5() {
        return Paths.line(intake2, move5).constant(move5);
    }

    public Path shooter3() {
        return Paths.curve(move5, shooter3Control1, shooter3).reverseTangent();
    }

    public Path park() {
        return Paths.curve(shooter3, parkControl1, park).constant(park);
    }
}