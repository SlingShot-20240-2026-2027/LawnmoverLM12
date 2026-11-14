package org.firstinspires.ftc.teamcode.Auton.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.frontRightName.set("fr");
        c.backLeftName.set("bl");
        c.backRightName.set("br");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(2.014597644956093);
        c.yPodOffset.set(-5.210694440706509);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.4601799208540868);
                Controller secondaryTranslationalForward = Controller.proportional(0.17002418452163687);
                Controller primaryTranslationalLateral = Controller.proportional(1.3231912336026361);
                Controller secondaryTranslationalLateral = Controller.proportional(0.4888838044952455);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01487723677914678));
                c.brake.set(Controller.proportionalFeedforward(0.012645651262274763));

                c.headingFeedback.set(Controller.proportional(8.103338401895094));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05152519023223146, 0.008248530720568269));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11813618621257015, 0.04867566093132077));
                c.quadraticBrakeCoefficients.set(Matrix.diag(9.797042964410439E-4, 0.00214614867148033));

                c.maxAchievableForwardVelocity.set(61.87183818490284);
                c.maxAchievableStrafeVelocity.set(46.127263014892335);
                c.naturalForwardDeceleration.set(39.50310811475273);
                c.naturalStrafeDeceleration.set(82.66332553640858);
            }
    );
}