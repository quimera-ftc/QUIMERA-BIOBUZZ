package org.firstinspires.ftc.teamcode.pedro;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
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
    private static PoseFactory p = PoseFactory.degrees();

    public static class VoltageConstants {
        public static double NOMINAL_VOLTAGE = 11.50;
    }

    public static class IntakeCoefficients {
        public static double intakeShooting = 0.75;
        public static double intakeCollecting = 1.0;
        public static double intakeIdle = 0;
    }
    @Config
    public static class ShooterCoefficients {

        public static double kS = 0.1756102778;
        public static double kV = 0.0003752448739;
        public static double kP = 0.0005;
        public static double READY_TOL = 50;
        public static double IDLE_TPS = 900;
        public static double ALVO_TPS = 1210;
        public static double ALVO_TPS_LADO = 1235;
    }

    public static class FieldPoses {
        public static final Pose topNestBLUE = p.of(84, 84,0);
        public static final Pose bottomNestBLUE = p.of(84, 57.5,0);
        public static final Pose topNestRED = p.of(57.5, 84,0);
        public static final Pose bottomNestRED = p.of(57.5, 57.5, 0);

        public static final Pose topParkBLUE = p.of(132.12685185185188, 51.289675925925924, 270);
        public static final Pose bottomParkBLUE = p.of(129.45, 17.78, 90);
    }

    public enum Alliance {
        RED,
        BLUE
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.frontRightName.set("fr");
        c.backLeftName.set("bl");
        c.backRightName.set("br");

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-0.14633333589148334);
        c.yPodOffset.set(0.18838323007418417);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(

            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.4519073567755977);
                Controller secondaryTranslationalForward = Controller.proportional(0.16696769314161838);
                Controller primaryTranslationalLateral = Controller.proportional(0.960617858083); //0.960617858083
                Controller secondaryTranslationalLateral = Controller.proportional(0.375387236913); //0.475387236913

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.008542520646834938));
                c.brake.set(Controller.proportionalFeedforward(0.007261142549809697));

                c.headingFeedback.set(Controller.proportional(2.002584212767482)); //2.602584212767482
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.16609097675135914, 0.006894017463738513));

                c.linearBrakeCoefficients.set(Matrix.diag(0.1034012946731598, 0.044359529102128675));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0031053216462230996, 0.014177042976344401));//0.0031053216462230996 //0.014177042976344401

                c.maxAchievableForwardVelocity.set(60.80197535823279);
                c.maxAchievableStrafeVelocity.set(45.001169530401384);
                c.naturalForwardDeceleration.set(26.726102036766637);
                c.naturalStrafeDeceleration.set(48.77118960818211); //
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}