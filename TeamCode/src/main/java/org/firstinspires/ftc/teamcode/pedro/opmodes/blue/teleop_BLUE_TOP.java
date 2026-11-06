
package org.firstinspires.ftc.teamcode.pedro.opmodes.blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.math.TabelaDisparo;
import org.firstinspires.ftc.teamcode.pedro.math.VoltageCompensator;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Agropecuaria;
import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Zones;
import org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.IntakeConstants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.ShooterCoefficients;

import java.util.List;

@TeleOp(name="TELEOP_BLUE_TOP", group="Blue")
public class teleop_BLUE_TOP extends OpMode {
    Intake intake;
    Agropecuaria agro;
    DriveTrain driveTrain;
    Shooter shooter;
    Turret turret;
    VoltageCompensator voltageController;
    boolean flower = false;
    boolean collecting = false;
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
        shooter = new Shooter(hardwareMap);
        voltageController = new VoltageCompensator(hardwareMap);
        turret.resetEncoder();
        agro.up();
        follower.setPose(startPose);

        zones = new Zones(Constants.ShooterCoefficients.ALVO_TPS)
                .addZone(50.96, 91.14, 105.14, 137.71, ShooterCoefficients.ALVO_TPS)
                .addZone(9.14, 50.96, 105.14, 137.71, ShooterCoefficients.ALVO_TPS_LADO)
                .addZone(91.14, 132.96, 105.14, 137.71, ShooterCoefficients.ALVO_TPS_LADO);

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

        follower.update();

        driveTrain.driveStick();

        Pose currentPose = follower.pose();
        turret.update(currentPose);

        if (gamepad1.aWasPressed()) {
            double coef = shooter.isShooting
                    ? IntakeConstants.intakeShooting
                    : IntakeConstants.intakeCollecting;

            intake.toggle(coef);
        }

        if (gamepad1.yWasPressed()) {
            intake.toggleReversed();
        }

        double distanciaGol = turret.getDistanciaGol();
        double alvoTabela = TabelaDisparo.tpsParaDistancia(distanciaGol);

        if (gamepad1.right_trigger > 0.5) {
            shooter.setAlvo(alvoTabela);
        } else {
            shooter.setAlvo(0);
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


        shooter.update();

        shooter.telemetriaTeste(telemetry);
        telemetry.addData("distancia gol (in)", "%.1f", distanciaGol);
        telemetry.addData("alvo da tabela (tps)", "%.0f", alvoTabela);
        telemetry.update();
    }


}