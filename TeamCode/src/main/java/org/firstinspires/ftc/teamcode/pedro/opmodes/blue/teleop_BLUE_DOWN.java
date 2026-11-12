package org.firstinspires.ftc.teamcode.pedro.opmodes.blue;

import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.CORRECTION_RATIO;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.MANUAL_POWER;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
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

@TeleOp(name="TELEOP_BLUE_DOWN", group="Blue")
public class teleop_BLUE_DOWN extends OpMode {
    Intake intake;
    Servos servos;
    DriveTrain driveTrain;
    Shooter shooter;
    Turret turret;
    VoltageCompensator voltageController;
    Pose startPose = new Pose(129.45, 17.79, Math.toRadians(90));
    Follower follower;
    List<LynxModule> allHubs;
    Zones zones;

    @Override
    public void init() {

        follower = Constants.create(hardwareMap);
        turret = new Turret(hardwareMap, startPose, Constants.Alliance.BLUE);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
        servos = new Servos(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, intake);
        voltageController = new VoltageCompensator(hardwareMap);

        turret.resetEncoder();
        follower.setPose(startPose);

        zones = new Zones(ShooterConstants.IDLE_TPS)
                .addZone(57.88,107.45,95.41,144.00, ShooterConstants.ALVO_TPS)
                .addZone(107.45,141.50,95.41,138.87, ShooterConstants.ALVO_TPS_LADO)
                .addZone(0.00, 57.88, 95.41, 144.00, ShooterConstants.ALVO_TPS_AWAY)

                .addZone(57.88, 107.45, 0.00, 48.59, ShooterConstants.ALVO_TPS)
                .addZone(107.45, 141.50, 5.13, 48.59, ShooterConstants.ALVO_TPS_LADO)
                .addZone(0.00, 57.88, 0.00, 48.59, ShooterConstants.ALVO_TPS_AWAY);

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
        follower.update();
        Scheduler.execute();
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        Pose currentPose = follower.pose();
        turret.update(currentPose);

        if (gamepad1.aWasPressed()) {
            if (!shooter.nectar) {
                if (shooter.isShooting) {
                    intake.toggle(IntakeConstants.intakeShooting);
                } else {
                    intake.toggle(IntakeConstants.intakeCollecting);
                }
            } else {
                intake.toggle(IntakeConstants.intakeBIGBALL);
            }


        }

        if (gamepad1.yWasPressed()) {
            intake.toggleReversed(IntakeConstants.intakeCollecting);
        }

        if (gamepad1.bWasPressed()) {
            shooter.shootNectar(!shooter.nectar);

        }

        if (gamepad1.xWasPressed()) {
            intake.collectFlower(!intake.flower);
        }

        if (gamepad1.right_trigger > 0.5) {
            if (!shooter.nectar) {
                shooter.setAlvo(zones.getTPS(currentPose.x(), currentPose.y()));
                shooter.setLock(false);
            } else {
                shooter.setAlvo(ShooterConstants.ALVO_TPS_BIGBALL);
                shooter.setLock(false);
            }
        } else {
            shooter.setAlvo(0);
            shooter.setLock(true);

        }

        if (gamepad1.right_bumper) {
            turret.setManualPower(MANUAL_POWER);
        } else if (gamepad1.left_bumper) {
            turret.setManualPower(-MANUAL_POWER);
        } else turret.setManualPower(0);

        if (gamepad1.dpadDownWasPressed()) {
            if (turret.getMode() == Turret.Mode.MANUAL) {
                turret.setMode(Turret.Mode.TRACKING);
            } else turret.setMode(Turret.Mode.MANUAL);
        }

        if (gamepad1.dpad_right) {
            turret.correct(CORRECTION_RATIO);
        } else if (gamepad1.dpad_left) {
            turret.correct(-CORRECTION_RATIO);
        }

        shooter.update();
        driveTrain.driveStick();
        intake.update();
    }


}
