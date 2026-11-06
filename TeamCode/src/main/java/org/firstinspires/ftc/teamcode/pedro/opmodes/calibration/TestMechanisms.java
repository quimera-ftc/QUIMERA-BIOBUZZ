package org.firstinspires.ftc.teamcode.pedro.opmodes.calibration;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.math.TabelaDisparo;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Agropecuaria;
import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Zones;
import org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.IntakeConstants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.ShooterCoefficients;

import java.util.List;

@TeleOp(name="TEST MECHANISMS", group="Tests")
public class TestMechanisms extends OpMode {
    Intake intake;
    DriveTrain driveTrain;
    Turret turret;
    Shooter shooter;
    Agropecuaria agro;
    Follower follower;
    Pose startPose = new Pose(132.6505617977528, 133.92247191011242, Math.toRadians(270));
    boolean flower = true;

    double potenciaTeste = 0.0;
    boolean dpadCimaAntes = false;
    boolean dpadBaixoAntes = false;
    List<LynxModule> allHubs;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        intake = new Intake(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
        shooter = new Shooter(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, startPose, Constants.Alliance.BLUE);
        agro = new Agropecuaria(hardwareMap);
        agro.up();
        follower.setPose(new Pose(132.6505617977528, 133.92247191011242, Math.toRadians(270)));

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        follower.update();

        Pose currentPose = follower.pose();
        turret.update(currentPose);

        if (gamepad1.aWasPressed()) {
            if(shooter.isShooting) {
                intake.toggle(IntakeConstants.intakeShooting);
            } else {
                intake.toggle(IntakeConstants.intakeCollecting);
            }

        }

        if (gamepad1.yWasPressed()) {
            intake.toggleReversed();
        }

        if (gamepad1.xWasPressed()) {

            if (!flower) {
                agro.up();
            } else agro.down();

            flower = !flower;

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
        driveTrain.driveStick();

        shooter.telemetriaTeste(telemetry); // TELEMETRIA PRA ANOTAR COISAS DE TESTE DO SHOOTER
        telemetry.addData("intake", intake.isOn() ? "LIGADO" : "DESLIGADO");
        telemetry.addData("shooter", shooter.isOn() ? "LIGADO" : "DESLIGADO");
        telemetry.addData("turret", turret.getMode());
        telemetry.addData("distancia gol (in)", "%.1f", distanciaGol);
        telemetry.addData("alvo da tabela (tps)", "%.0f", alvoTabela);
        telemetry.addLine("--ODOMETRIA--");
        telemetry.addData("x", currentPose.x());
        telemetry.addData("y", currentPose.y());
        telemetry.addData("heading", Math.toDegrees(currentPose.heading()));
        telemetry.addData("voltage","%.2f V", getVoltage());

        telemetry.update();
    }

    public double getVoltage() {
        double minVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                minVoltage = Math.min(minVoltage, voltage);
            }
        }
        return minVoltage;
    }
}
