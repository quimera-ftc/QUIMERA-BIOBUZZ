
package org.firstinspires.ftc.teamcode.pedro.opmodes.blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.math.VoltageCompensator;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Agropecuaria;
import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Servos;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Zones;
import org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.IntakeConstants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.ShooterCoefficients;

import java.util.List;

@TeleOp(name="TELEOP_RED_TOP", group="Red")
public class teleop_RED_TOP extends OpMode {
    Intake intake;
    Agropecuaria agro;
    DriveTrain driveTrain;
    Shooter shooter;
    Turret turret;
    Servos servos;
    VoltageCompensator voltageController;
    Pose startPose = new Pose(132.12, 51.28, Math.toRadians(270));
    Follower follower;
    List<LynxModule> allHubs;
    Zones zones;

    @Override
    public void init() {

        follower = Constants.create(hardwareMap);
        turret = new Turret(hardwareMap, startPose, Constants.Alliance.BLUE);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
        agro = new Agropecuaria(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, intake);
        voltageController = new VoltageCompensator(hardwareMap);
        servos = new Servos(hardwareMap);

        turret.resetEncoder();
        agro.up();
        follower.setPose(startPose);

        zones = new Zones(Constants.ShooterCoefficients.ALVO_TPS)
                .addZone(50.96, 91.14, 105.14, 137.71, ShooterCoefficients.ALVO_TPS)
                .addZone(9.14, 50.96, 105.14, 137.71, ShooterCoefficients.ALVO_TPS_LADO)
                .addZone(91.14, 132.96, 105.14, 137.71, ShooterCoefficients.ALVO_TPS_LADO)

                .addZone(50.96, 91.14, 6.29, 38.86, ShooterCoefficients.ALVO_TPS)
                .addZone(9.14, 50.96, 6.29, 38.86, ShooterCoefficients.ALVO_TPS_LADO)
                .addZone(91.14, 132.96, 6.29, 38.86, ShooterCoefficients.ALVO_TPS_LADO);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void start() {

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
            servos.notLocking();
        } else {
            shooter.setAlvo(0);
            servos.locking();
        }


        if (gamepad1.right_bumper) {
            turret.setManualPower(0.75);
        } else if (gamepad1.left_bumper) {
            turret.setManualPower(-0.75);
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

        shooter.telemetriaTeste(telemetry);
        telemetry.addData("intake", "%s (potência %.2f)", intake.isOn() ? "LIGADO" : "DESLIGADO", intake.intake.getPower());
        telemetry.update();
    }


}