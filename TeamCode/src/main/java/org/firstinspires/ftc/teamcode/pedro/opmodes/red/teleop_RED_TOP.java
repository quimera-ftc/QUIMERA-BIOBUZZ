package org.firstinspires.ftc.teamcode.pedro.opmodes.red;

import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.MANUAL_POWER;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.math.VoltageCompensator;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Servos;
import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Zones;
import org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.IntakeConstants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.ShooterConstants;

import java.util.List;

@TeleOp(name="TELEOP_RED_TOP", group="Red")
public class teleop_RED_TOP extends OpMode {
    Intake intake;
    Servos servos;
    DriveTrain driveTrain;
    Shooter shooter;
    Turret turret;
    VoltageCompensator voltageController;
    Pose startPose = new Pose(132.12, 51.28, Math.toRadians(270)); //todo: corrigir pose
    Follower follower;
    List<LynxModule> allHubs;
    Zones zones;

    @Override
    public void init() {

        follower = Constants.create(hardwareMap);
        turret = new Turret(hardwareMap, startPose, Constants.Alliance.RED);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
        servos = new Servos(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, intake);
        voltageController = new VoltageCompensator(hardwareMap);

        turret.resetEncoder();
        intake.collectFlower(false);
        follower.setPose(startPose);

        zones = new Zones(ShooterConstants.ALVO_TPS)
                .addZone(34.20, 83.24, 96.00, 144.00, ShooterConstants.ALVO_TPS)
                .addZone(0.00, 34.20, 96.00, 144.00, ShooterConstants.ALVO_TPS_LADO)
                .addZone(83.63, 144.00, 96.00, 144.00, ShooterConstants.ALVO_TPS_AWAY)

                .addZone(60.76, 109.80, 96.00, 144.00, ShooterConstants.ALVO_TPS)
                .addZone(109.80, 144.00, 96.00, 144.00, ShooterConstants.ALVO_TPS_LADO)
                .addZone(0.00, 60.37, 96.00, 144.00, ShooterConstants.ALVO_TPS_AWAY);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void start() {
        turret.setMode(Turret.Mode.MANUAL);
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        Pose currentPose = follower.pose();
        turret.update(currentPose);

        if (gamepad1.aWasPressed()) {
            double coef = shooter.isShooting
                    ? IntakeConstants.intakeShooting
                    : IntakeConstants.intakeCollecting;

            intake.toggle(coef);
        }

        if (gamepad1.yWasPressed()) {
            intake.toggleReversed(IntakeConstants.intakeCollecting);
        }


        if (gamepad1.right_trigger > 0.5) {
            shooter.setAlvo(zones.getTPS(currentPose.x(), currentPose.y()));

        } else {
            shooter.setAlvo(0);

        }


        if (gamepad1.right_bumper) {
            turret.setManualPower(MANUAL_POWER);
        } else if (gamepad1.left_bumper) {
            turret.setManualPower(-MANUAL_POWER);
        } else turret.setManualPower(0);

        if (gamepad1.dpadRightWasPressed()) {

            if (turret.getMode() == Turret.Mode.MANUAL) {
                turret.setMode(Turret.Mode.TRACKING);
            } else turret.setMode(Turret.Mode.MANUAL);

        }

        follower.update();
        driveTrain.driveStick();
        shooter.update();
        intake.update();

        telemetry.update();
    }


}