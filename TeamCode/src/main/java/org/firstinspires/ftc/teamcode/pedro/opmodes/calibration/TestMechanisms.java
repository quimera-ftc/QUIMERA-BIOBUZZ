package org.firstinspires.ftc.teamcode.pedro.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.math.TurretController;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Servos;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Servos;
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
    Servos servos;
    Follower follower;
    Zones zones;
    Pose startPose = new Pose(132.6505617977528, 133.92247191011242, Math.toRadians(270));
    boolean flower = true;

    double potenciaTeste = 0.0;
    boolean dpadCimaAntes = false;
    boolean dpadBaixoAntes = false;
    List<LynxModule> allHubs;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.create(hardwareMap);
        intake = new Intake(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
        shooter = new Shooter(hardwareMap, telemetry, intake);
        turret = new Turret(hardwareMap, startPose, Constants.Alliance.BLUE);
        servos = new Servos(hardwareMap);

        follower.setPose(new Pose(132.6505617977528, 133.92247191011242, Math.toRadians(270)));
        turret.resetEncoder();

        zones = new Zones(ShooterCoefficients.IDLE_TPS)
                .addZone(50.96,  91.14, 105.14, 137.71, ShooterCoefficients.ALVO_TPS)
                .addZone( 9.14,  50.96, 105.14, 137.71, ShooterCoefficients.ALVO_TPS_LADO)
                .addZone(91.14, 132.96, 105.14, 137.71, ShooterCoefficients.ALVO_TPS_LADO)

                .addZone(50.96,  91.14, 6.29, 38.86, ShooterCoefficients.ALVO_TPS)
                .addZone( 9.14,  50.96, 6.29, 38.86, ShooterCoefficients.ALVO_TPS_LADO)
                .addZone(91.14, 132.96, 6.29, 38.86, ShooterCoefficients.ALVO_TPS_LADO);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void start() {
        servos.agroUp();
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

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
            intake.toggleReversed(IntakeConstants.intakeCollecting);
        }

        if (gamepad1.xWasPressed()) {

            if (!flower) {
                servos.agroUp();
            } else {
                servos.agroDown();
            }

            flower = !flower;

        }

        if (gamepad1.right_trigger > 0.5) {
            shooter.setAlvo(zones.getTPS(currentPose.x(), currentPose.y()));
            servos.open();
        } else {
            shooter.setAlvo(0);
            servos.close();

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
        intake.update();
        follower.update();

        Pose turretPose = TurretController.getTurretPose(currentPose);
        double target = TurretController.computeTargetAngle(turretPose, TurretController.getTurretPose(turretPose));

        shooter.telemetriaTeste(telemetry); // TELEMETRIA PRA ANOTAR COISAS DE TESTE DO SHOOTER
        telemetry.addData("intake", "%s (potência %.2f)", intake.isOn() ? "LIGADO" : "DESLIGADO", intake.intake.getPower());
        telemetry.addData("shooter", shooter.isOn() ? "LIGADO" : "DESLIGADO");
        telemetry.addData("turret", turret.getMode());
        telemetry.addLine("----------ODOMETRIA-----------");
        telemetry.addData("x", currentPose.x());
        telemetry.addData("y", currentPose.y());
        telemetry.addData("heading", Math.toDegrees(currentPose.heading()));
        telemetry.addData("voltage","%.2f V", getVoltage());
        telemetry.addLine("------------TURRET------------");
        telemetry.addData("mode", turret.getMode());
        telemetry.addData("Turret X", turretPose.x());
        telemetry.addData("Turret Y", turretPose.y());
        telemetry.addData("target", target);
        telemetry.addData("deslocamento (Raw)", turret.getRawAngle());
        telemetry.addData("Erro Atual", turret.getDistanceToTarget());
        telemetry.addData("lime offset", turret.getLimeLightOffset());
        telemetry.addData("na mosca", turret.aligned() ? "sim" : "nao");
        telemetry.addData("tempo de ciclo (dt)", turret.getDt());

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
