package org.firstinspires.ftc.teamcode.BareAuto;

import static org.firstinspires.ftc.teamcode.BareAuto.sdk.Actions.sequential;
import static org.firstinspires.ftc.teamcode.BareAuto.sdk.Actions.waitMs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BareAuto.sdk.Action;
import org.firstinspires.ftc.teamcode.BareAuto.sdk.Actions;
import org.firstinspires.ftc.teamcode.BareAuto.sdk.Drive;
import org.firstinspires.ftc.teamcode.BareAuto.sdk.Intake;
import org.firstinspires.ftc.teamcode.BareAuto.sdk.Shooter;
import org.firstinspires.ftc.teamcode.BareAuto.sdk.Transfer;

@Autonomous(name = "SoloAuto SDK", group = "Autonomous")
public class SoloAutoSDK extends LinearOpMode {

    private static final double START_X = 55.92;
    private static final double START_Y = 9.218;
    private static final double START_HEADING = 90;

    private static final String DEPLOYABLE_NAME = "3";
    private static final double DEPLOY_UP = 0.0;

    private Drive drive;
    private Shooter shooter;
    private Transfer transfer;
    private Intake intake;

    public Action autoRoutine() {
        return sequential(
                intake.in(),
                shooter.on(),
                waitMs(3000),
                transfer.transfer(),
                shooter.idle(),
                drive.moveTo(40.81, 18.14, 90),
                drive.turnTo(270),
                drive.moveTo(8.75, 29.39, 270),
                drive.moveTo(8.75, 8.78, 270),
                drive.moveTo(25.93, 61.67, 270),
                drive.moveTo(13.69, 95.71, 270),
                intake.idle()
        );
    }

    @Override
    public void runOpMode() {
        drive = new Drive(hardwareMap, START_X, START_Y, START_HEADING);
        shooter = new Shooter(hardwareMap);
        transfer = new Transfer(hardwareMap);
        intake = new Intake(hardwareMap);
        hardwareMap.get(Servo.class, DEPLOYABLE_NAME).setPosition(DEPLOY_UP);

        telemetry.addLine("Ready. Robot at the start spot, facing 90 deg. Keep it still.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        drive.resetHeading();
        Actions.run(this, autoRoutine(), this::everyLoop);

        while (opModeIsActive()) {
            everyLoop();
        }

        drive.stopMotors();
        shooter.stop();
        transfer.setState(Transfer.State.IDLE);
        transfer.closeGate();
        intake.setState(Intake.State.IDLE);
    }

    private void everyLoop() {
        shooter.update();
        drive.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        telemetry.addData("Transfer", transfer.getState());
        telemetry.addData("Intake", intake.getState());
        telemetry.update();
    }
}
