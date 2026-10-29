package org.firstinspires.ftc.teamcode.pedro.opmodes.autonomous.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.TowerCLAUDE;

/**
 * Calibracao da torreta. CENTRALIZE a torreta (0 = frente) antes do init.
 *
 * gamepad1: dirige o robo normalmente
 *   A       mede o kS (sobe a potencia devagar ate a torreta comecar a girar, nos dois sentidos)
 *   X/Y/B   degrau para -45 / 0 / +45 graus (mostra overshoot, tempo e re-correcoes)
 *   dpad >  liga/desliga o rastreio da colmeia (gire o robo no lugar para acertar o kV)
 *   LB      liga/desliga o feedforward (para ver a diferenca girando o robo)
 *
 * gamepad2: muda os ganhos na hora (anote os valores e passe para o Tower.java no fim)
 *   dpad cima/baixo   kP +-        dpad dir/esq   kD +-
 *   Y / A             kV +-        RB / LB        kS +-
 *   B / X             kA +-
 */
@TeleOp(name = "Test_TOWER", group = "tests")
public class Test_TOWERCLAUDE extends OpMode {

    private Follower follower;
    private DriveTrain driveTrain;
    private TowerCLAUDE tower;
    private DcMotorEx towerMotor;
    private VoltageSensor battery;

    // ---- medicao do kS ----
    private boolean measuringKs = false;
    private int ksDirection = 1;
    private double ksPositive = Double.NaN, ksNegative = Double.NaN;
    private final ElapsedTime ksTimer = new ElapsedTime();
    private static final double KS_RAMP = 0.05;  // potencia por segundo
    private static final double KS_PAUSE = 0.6;  // s parado antes de cada rampa

    // ---- teste de degrau ----
    private boolean stepRunning = false, stepEverRun = false;
    private double stepTarget, stepStart, stepOvershoot, stepSettleTime = Double.NaN;
    private int stepRecorrections;
    private boolean wasSettled;
    private final ElapsedTime stepTimer = new ElapsedTime();

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(132.6505617977528, 133.92247191011242, Math.toRadians(270)));
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
        tower = new TowerCLAUDE(hardwareMap, telemetry);
        towerMotor = hardwareMap.get(DcMotorEx.class, "tower");
        battery = hardwareMap.voltageSensor.iterator().next();

        telemetry.addLine("Centralize a torreta (0 = frente) ANTES do init.");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.pose();

        adjustGains();

        if (measuringKs) {
            measureKs();
        } else {
            if (gamepad1.aWasPressed()) {
                startKs();
            } else if (gamepad1.xWasPressed()) {
                startStep(-45);
            } else if (gamepad1.yWasPressed()) {
                startStep(0);
            } else if (gamepad1.bWasPressed()) {
                startStep(45);
            }

            if (gamepad1.leftBumperWasPressed()) {
                TowerCLAUDE.FEEDFORWARD = !TowerCLAUDE.FEEDFORWARD; // compare girando o robo com e sem
            }

            if (gamepad1.dpadRightWasPressed()) {
                stepRunning = false;
                if (tower.isTracking()) {
                    tower.stop();
                } else {
                    tower.track(pose.y() >= 70.5 ? TowerCLAUDE.TrackingPose.TOP_GOAL_BLUE : TowerCLAUDE.TrackingPose.BOTTOM_GOAL_BLUE);
                }
            }

            if (!measuringKs) {
                tower.update(pose);
                watchStep();
                showTracking();
            }
        }

        driveTrain.driveStick();
        showGains();
        telemetry.update();
    }

    // ===================== kS =====================

    private void startKs() {
        tower.stop();
        stepRunning = false;
        measuringKs = true;
        ksDirection = 1;
        ksPositive = Double.NaN;
        ksNegative = Double.NaN;
        ksTimer.reset();
    }

    private void measureKs() {
        double t = ksTimer.seconds() - KS_PAUSE;
        double angle = towerMotor.getCurrentPosition() / TowerCLAUDE.TICKS_PER_DEGREE;
        double speed = Math.abs(towerMotor.getVelocity() / TowerCLAUDE.TICKS_PER_DEGREE);

        telemetry.addLine(">> Medindo kS (" + (ksDirection > 0 ? "esquerda" : "direita") + ")... B cancela");

        if (gamepad1.bWasPressed()) {
            towerMotor.setPower(0);
            measuringKs = false;
            return;
        }
        if (t < 0) { // pausa parada antes da rampa
            towerMotor.setPower(0);
            return;
        }

        double p = KS_RAMP * t;
        boolean moved = speed > 10;                               // graus/s
        boolean unsafe = p > 0.4 || Math.abs(angle) > TowerCLAUDE.MAX_SPIN - 15;

        if (moved || unsafe) {
            towerMotor.setPower(0);
            // potencia em "volts de 12 V", que e como o Tower.java usa (ele compensa a bateria)
            double normalized = moved ? p * battery.getVoltage() / TowerCLAUDE.NOMINAL_VOLTAGE : Double.NaN;
            if (ksDirection > 0) {
                ksPositive = normalized;
                ksDirection = -1;
                ksTimer.reset();
            } else {
                ksNegative = normalized;
                measuringKs = false;
                if (!Double.isNaN(ksPositive) && !Double.isNaN(ksNegative)) {
                    // a potencia que "desgruda" e maior que a que mantem girando: usa ~80% dela
                    // (kS alto demais faz a torreta "sambar"; baixo demais so deixa um errinho)
                    TowerCLAUDE.kS = 0.8 * (ksPositive + ksNegative) / 2.0;
                }
            }
            return;
        }
        towerMotor.setPower(ksDirection * p);
        telemetry.addData("potencia", "%.3f", p);
    }

    // ===================== degrau =====================

    private void startStep(double degrees) {
        tower.goTo(degrees);
        stepTarget = degrees;
        stepStart = tower.getAngle();
        stepOvershoot = 0;
        stepSettleTime = Double.NaN;
        stepRecorrections = 0;
        wasSettled = false;
        stepRunning = Math.abs(stepTarget - stepStart) > 3;
        stepEverRun = stepEverRun || stepRunning;
        stepTimer.reset();
    }

    private void watchStep() {
        if (!stepRunning) {
            return;
        }
        double direction = Math.signum(stepTarget - stepStart);
        stepOvershoot = Math.max(stepOvershoot, (tower.getAngle() - stepTarget) * direction);

        boolean settledNow = tower.isSettled();
        if (settledNow && Double.isNaN(stepSettleTime)) {
            stepSettleTime = stepTimer.seconds();
        }
        if (wasSettled && !settledNow) {
            stepRecorrections++; // assentou e depois voltou a corrigir sozinha = "samba"
        }
        wasSettled = settledNow;

        if (stepTimer.seconds() > 4) {
            stepRunning = false;
        }
    }

    private void showTracking() {
        if (stepEverRun) {
            telemetry.addData("Degrau: passou do alvo", "%.1f graus", stepOvershoot);
            telemetry.addData("Degrau: assentou em", Double.isNaN(stepSettleTime) ? "-" : String.format("%.2f s", stepSettleTime));
            telemetry.addData("Degrau: re-correcoes (samba)", stepRecorrections);
        }

        if (tower.isTracking()) {
            double targetVel = tower.getTargetVelocity();
            if (Math.abs(targetVel) > 30) {
                // positivo = a torreta esta ATRAS do alvo -> aumente kV (ou kA nas arrancadas)
                telemetry.addData("Atraso da mira", "%.1f graus", tower.getError() * Math.signum(targetVel));
            } else {
                telemetry.addData("Atraso da mira", "- (gire o robo no lugar)");
            }
            telemetry.addData("Robo girando", "%.0f graus/s", tower.getRobotOmega());
        }
    }

    // ===================== ganhos =====================

    private void adjustGains() {
        if (gamepad2.dpadUpWasPressed()) TowerCLAUDE.kP += 0.002;
        if (gamepad2.dpadDownWasPressed()) TowerCLAUDE.kP = Math.max(0, TowerCLAUDE.kP - 0.002);
        if (gamepad2.dpadRightWasPressed()) TowerCLAUDE.kD += 0.0005;
        if (gamepad2.dpadLeftWasPressed()) TowerCLAUDE.kD = Math.max(0, TowerCLAUDE.kD - 0.0005);
        if (gamepad2.yWasPressed()) TowerCLAUDE.kV += 0.00002;
        if (gamepad2.aWasPressed()) TowerCLAUDE.kV = Math.max(0, TowerCLAUDE.kV - 0.00002);
        if (gamepad2.rightBumperWasPressed()) TowerCLAUDE.kS += 0.005;
        if (gamepad2.leftBumperWasPressed()) TowerCLAUDE.kS = Math.max(0, TowerCLAUDE.kS - 0.005);
        if (gamepad2.bWasPressed()) TowerCLAUDE.kA += 0.00005;
        if (gamepad2.xWasPressed()) TowerCLAUDE.kA = Math.max(0, TowerCLAUDE.kA - 0.00005);
    }

    private void showGains() {
        telemetry.addLine("---- ganhos (passe para o Tower.java) ----");
        telemetry.addData("Feedforward (LB)", TowerCLAUDE.FEEDFORWARD ? "LIGADO" : "desligado");
        telemetry.addData("kP", "%.4f", TowerCLAUDE.kP);
        telemetry.addData("kD", "%.4f", TowerCLAUDE.kD);
        telemetry.addData("kV", "%.5f", TowerCLAUDE.kV);
        telemetry.addData("kS", "%.3f   (medido: esq %.3f / dir %.3f)", TowerCLAUDE.kS, ksPositive, ksNegative);
        telemetry.addData("kA", "%.5f", TowerCLAUDE.kA);
    }
}
