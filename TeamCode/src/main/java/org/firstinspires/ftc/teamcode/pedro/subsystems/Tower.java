package org.firstinspires.ftc.teamcode.pedro.subsystems;

import androidx.annotation.NonNull;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Tower {
    double gearRatio = 50.0 / 15.0;
    double ticksPerRev = 28.0 * (76.0 / 21.0) * gearRatio; // HD Hex = 28 ticks/volta; redução "4:1" é na real 76/21 (~3.62:1)
    double ticksPerDegree = ticksPerRev / 360.0;

    private enum TowerState {
        IDLE,
        TRACKING,
        LOCKED,
        ROTATING
    }

    public enum TrackingPose {
        TOP_GOAL_BLUE,
        BOTTOM_GOAL_BLUE,
        TOP_GOAL_RED,
        BOTTOM_GOAL_RED
    }

    public enum Direction {
        RIGHT,
        LEFT
    }

    private final DcMotorEx tower;
    private final double towerPower = 0.6, maxSpin = 150;

    double kP = 0.05;          // potência
    double kI = 0.001;            // [era kL] deixe em 0 até P + D estarem bons
    double kD = 0.0025;         // amortecimento
    double kS = 0.075;           // potência mínima p/ vencer o atrito
    double maxPower = 0.8;      // teto de potência durante o tracking
    double toleranceDeg = 1.5;  // zona morta (1 tick ≈ 1.07°, então ~1,5 ticks)
    double integralLimit = 20;  // anti-windup do termo I (°·s)

    private final double[] offsets = new double[] {0, -1.772};

    private TowerState state = TowerState.IDLE, lastState = TowerState.IDLE;
    private final Pose topNestBLUE = new Pose(84, 84);
    private final Pose bottomNestBLUE = new Pose(84, 59);
    private final Pose topNestRED = new Pose(60, 84);
    private final Pose bottomNestRED = new Pose(60, 59);
    private Pose trackedPose = topNestBLUE, currentPose;
    private final Telemetry telemetry;
    private int rotatingTo;
    private double lastError;
    private boolean firstLoop = true;   // [NOVO] evita "chute" da derivada no 1º ciclo
    ElapsedTime timer;
    double integral = 0;

    public Tower(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        tower = hardwareMap.get(DcMotorEx.class, "tower");
        timer = new ElapsedTime();
        this.telemetry = telemetry;

        reset();
    }

    public Tower(@NonNull HardwareMap hardwareMap) {
        tower = hardwareMap.get(DcMotorEx.class, "tower");
        timer = new ElapsedTime();
        telemetry = null;

        reset();
    }

    public void reset() {
        state = TowerState.IDLE;

        tower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tower.setPower(0);
        tower.setDirection(DcMotor.Direction.REVERSE);
        tower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tower.setTargetPositionTolerance(2);

        resetPid();   // [CORRIGIDO] antes só fazia timer.reset()
    }

    // [NOVO] Zera tudo que o PID "lembra". Chamar sempre que (re)entrar em TRACKING.
    private void resetPid() {
        integral = 0;
        lastError = 0;
        firstLoop = true;
        timer.reset();
    }

    public void rotate(int direction) {

        if (state == TowerState.LOCKED) {
            return;
        }

//        if (dislocation() >= maxSpin && direction == 1 ||
//                dislocation() <= -maxSpin && direction == -1) {
//            stop();
//            return;
//        }

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        tower.setPower(direction);

        rotatingTo = direction;
        state = TowerState.ROTATING;
    }

    // LEFT = anti-horário, RIGHT = horário
    public void rotate(@NonNull Direction direction) {
        rotate(direction == Direction.LEFT ? 1 : -1);
    }

    public void lock() {
        if (state == TowerState.LOCKED) {
            return;
        }
        lastState = state == TowerState.TRACKING ? TowerState.TRACKING : TowerState.IDLE;
        state = TowerState.LOCKED;

        tower.setTargetPosition(tower.getCurrentPosition());
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tower.setPower(towerPower);
    }

    public void unlock() {
        if (state != TowerState.LOCKED) {
            return;
        }

        state = lastState;
        tower.setPower(0);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (state == TowerState.TRACKING) {   // [NOVO] volta ao tracking "do zero"
            resetPid();
        }
    }


    public void update(@NonNull Pose pose) {
        currentPose = pose;
        fix();
    }

    public void printTelemetry() {
        if (telemetry != null) {
            telemetry.addData("estado torre", state);
            telemetry.addData("ang. torre", dislocation());
            telemetry.addData("erro torre", lastError);
        }
    }

    public void stop() {
        state = TowerState.IDLE;
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tower.setPower(0);
    }

    private void setMode(DcMotor.RunMode mode) {
        if (tower.getMode() != mode) {
            tower.setMode(mode);
        }
    }

    // [ATENÇÃO] Este método nunca é chamado por ninguém, então o limite de
    // rotação manual (maxSpin) não está valendo. No tracking o limite agora é
    // aplicado dentro do fix().
    private void enforceLimits() {
        double currentDislocation = dislocation();

        if (state == TowerState.ROTATING) {

            if (currentDislocation >= maxSpin && rotatingTo == 1 ||
                    currentDislocation <= -maxSpin && rotatingTo == -1)  {
                stop();
            }
        }
    }

    // =====================================================================
    //  fix(): controle de posição da torre (PID em graus + kS + zona morta)
    // =====================================================================
    private void fix() {
        if (state != TowerState.TRACKING) {
            return;
        }

        double dt = timer.seconds();
        if (dt < 0.001) {
            return;
        }
        timer.reset();
        dt = Math.min(dt, 0.1);

        double current = dislocation();
        double target = Range.clip(calculateTargetAngle(), -maxSpin, maxSpin);
        double error = target - current;

        if (firstLoop) {
            lastError = error;
            firstLoop = false;
        }

        if (Math.abs(error) <= toleranceDeg) {
            tower.setPower(0);
            integral = 0;
            lastError = error;
            return;
        }

        integral = Range.clip(integral + error * dt, -integralLimit, integralLimit);
        double derivative = (error - lastError) / dt;
        lastError = error;

        double P = kP * error;
        double I = kI * integral;
        double D = kD * derivative;
        double S = kS * Math.signum(error);

        double power = Range.clip(P + I + D + S, -maxPower, maxPower);
        tower.setPower(power);

        if (telemetry != null) {
            telemetry.addData("dt", dt);
            telemetry.addData("alvo (°)", target);
            telemetry.addData("ang. atual (°)", current);
            telemetry.addData("erro (°)", error);
            telemetry.addData("derivada (°/s)", derivative);
            telemetry.addData("integral", integral);
            telemetry.addData("P", P);
            telemetry.addData("I", I);
            telemetry.addData("D", D);
            telemetry.addData("S", S);
            telemetry.addData("power", power);
        }
    }

    public boolean isBusy() {
        return state != TowerState.IDLE;
    }
    public boolean isLocked() {
        return state == TowerState.LOCKED;
    }
    public boolean isTracking() {
        return state == TowerState.TRACKING;
    }
    public void track(@NonNull TrackingPose pose) {

        switch (pose) {
            case TOP_GOAL_BLUE:
                trackedPose = topNestBLUE;
                break;

            case BOTTOM_GOAL_BLUE:
                trackedPose = bottomNestBLUE;
                break;

            case TOP_GOAL_RED:
                trackedPose = topNestRED;
                break;

            case BOTTOM_GOAL_RED:
                trackedPose = bottomNestRED;
                break;
        }

        if (state == TowerState.LOCKED) {
            return;
        }

        if (state != TowerState.TRACKING) {   // [NOVO] só reseta ao ENTRAR em tracking
            resetPid();
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        state = TowerState.TRACKING;
    }
    public double dislocation() {
        int ticks = tower.getCurrentPosition();
        return ticks / ticksPerDegree;
    }

    // [NOVO] Ângulo ABSOLUTO que a torre deveria ter (graus, relativo à frente do
    // robô) para apontar para o alvo. 0° = frente do robô, positivo = anti-horário.
    public double calculateTargetAngle() {

        double headingRad = currentPose.heading();

        double turretX = currentPose.x() + (offsets[0] * Math.cos(headingRad) - offsets[1] * Math.sin(headingRad));
        double turretY = currentPose.y() + (offsets[0] * Math.sin(headingRad) + offsets[1] * Math.cos(headingRad));

        double deltaX = trackedPose.x() - turretX;
        double deltaY = trackedPose.y() - turretY;

        double theta = Math.toDegrees(Math.atan2(deltaY, deltaX));   // direção do alvo no campo

        return normalize(theta - Math.toDegrees(headingRad));
    }

    // Quantos graus AINDA FALTAM girar a partir da posição atual (erro relativo).
    // Mantido por compatibilidade; o fix() usa calculateTargetAngle().
    public double calculateCorrection() {
        return normalize(calculateTargetAngle() - dislocation());
    }

    private static double normalize(double degrees) {
        degrees %= 360;
        if (degrees > 180) degrees -= 360;
        if (degrees <= -180) degrees += 360;
        return degrees;
    }
}