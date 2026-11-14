package org.firstinspires.ftc.teamcode.__________;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Auton.RobotConstants;

@TeleOp(name = "SimpleTeleOp")
public class SimpleTeleop extends OpMode {

    /** Mec Define **/
    DcMotor leftFront, rightFront, leftBack, rightBack;

    /** Intake + Transfer **/
    private DcMotor intake, transfer;

    /** Shooter Flywheels **/
    private DcMotorEx flywheel1, flywheel2;

    /** Gate **/
    private Servo gate;

    /** Hood **/
    private Servo hood;

    /** Deployable **/
    private Servo deployable;

    @Override
    public void init() {
        /** Mec Init **/
        leftFront  = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_LEFT_NAME);
        rightFront = hardwareMap.get(DcMotor.class, RobotConstants.FRONT_RIGHT_NAME);
        leftBack   = hardwareMap.get(DcMotor.class, RobotConstants.BACK_LEFT_NAME);
        rightBack  = hardwareMap.get(DcMotor.class, RobotConstants.BACK_RIGHT_NAME);

        leftFront.setDirection(direction(RobotConstants.LEFT_DRIVE_REVERSED));
        leftBack.setDirection(direction(RobotConstants.LEFT_DRIVE_REVERSED));
        rightFront.setDirection(direction(RobotConstants.RIGHT_DRIVE_REVERSED));
        rightBack.setDirection(direction(RobotConstants.RIGHT_DRIVE_REVERSED));

        /** Intake / Transfer Init **/
        intake   = hardwareMap.get(DcMotor.class, RobotConstants.INTAKE_NAME);
        transfer = hardwareMap.get(DcMotor.class, RobotConstants.TRANSFER_NAME);

        intake.setDirection(direction(RobotConstants.INTAKE_REVERSED));
        transfer.setDirection(direction(RobotConstants.TRANSFER_REVERSED));

        /** Flywheel Init **/
        flywheel1 = hardwareMap.get(DcMotorEx.class, RobotConstants.FLYWHEEL1_NAME);
        flywheel2 = hardwareMap.get(DcMotorEx.class, RobotConstants.FLYWHEEL2_NAME);

        flywheel1.setDirection(direction(RobotConstants.FLYWHEEL1_REVERSED));
        flywheel2.setDirection(direction(RobotConstants.FLYWHEEL2_REVERSED));

        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(
                RobotConstants.FLYWHEEL_P, RobotConstants.FLYWHEEL_I,
                RobotConstants.FLYWHEEL_D, RobotConstants.FLYWHEEL_F);
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        /** Gate Init **/
        gate = hardwareMap.get(Servo.class, RobotConstants.GATE_NAME);
        gate.setPosition(RobotConstants.GATE_CLOSED);

        /** Hood Init **/
        hood = hardwareMap.get(Servo.class, RobotConstants.HOOD_NAME);
        hood.setPosition(RobotConstants.HOOD_UP);

        /** Deployable Init **/
        deployable = hardwareMap.get(Servo.class, RobotConstants.DEPLOY_NAME);
        deployable.setPosition(RobotConstants.DEPLOY_UP);
    }

    @Override
    public void loop() {
        /** ---------------- Mecanum ---------------- **/
        double drive  = -gamepad1.left_stick_y;
        double strafe =  -gamepad1.left_stick_x;
        double turn   =  gamepad1.right_stick_x;

        double leftFrontPower  = drive - strafe + turn;
        double rightFrontPower = drive + strafe - turn;
        double leftBackPower   = drive + strafe + turn;
        double rightBackPower  = drive - strafe - turn;

        double max = Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
        );

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        /** Hood held at a fixed position **/
        hood.setPosition(RobotConstants.HOOD_UP);

        /** ---------------- Intake (right bumper) & Transfer (always on) ---------------- **/
        intake.setPower(gamepad1.right_bumper ? RobotConstants.INTAKE_POWER : 0);

        if (gamepad1.left_trigger > 0.5) {
            transfer.setPower(-RobotConstants.TRANSFER_REVERSE_POWER);
        } else {
            transfer.setPower(RobotConstants.TRANSFER_POWER);
        }

        /** ---------------- Shooter Flywheels & Gate ---------------- **/
        flywheel1.setVelocity(RobotConstants.FLYWHEEL_VELOCITY);
        flywheel2.setVelocity(RobotConstants.FLYWHEEL_VELOCITY);
        gate.setPosition(gamepad1.right_trigger > 0.5 ? RobotConstants.GATE_OPEN : RobotConstants.GATE_CLOSED);

        /** ---------------- Deployable (hold left bumper) ---------------- **/
        boolean deployed = gamepad1.left_bumper;
        deployable.setPosition(deployed ? RobotConstants.DEPLOY_DOWN : RobotConstants.DEPLOY_UP);

        telemetry.addData("Deployed", deployed);
        telemetry.addData("Flywheel target", RobotConstants.FLYWHEEL_VELOCITY);
        telemetry.addData("Flywheel 1 vel", flywheel1.getVelocity());
        telemetry.addData("Flywheel 2 vel", flywheel2.getVelocity());
    }

    private static DcMotorSimple.Direction direction(boolean reversed) {
        return reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }
}
