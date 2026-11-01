package org.firstinspires.ftc.teamcode.pedro.subsystems;

import androidx.annotation.NonNull;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Turret {

    static double maxPower  = 1, manualPower = 0;

    // --- mecanico ---
    DcMotorEx motor;
    static double OFFSET_Y = 0.0;
    static double OFFSET_X = -1.772;
    double gearRatio = 50.0 / 15.0;
    double ticksPerRev = 28.0 * (76.0 / 21.0) * gearRatio; // HD Hex = 28 ticks/volta; redução "4:1" é na real 76/21 (~3.62:1)
    double ticksPerDegree = ticksPerRev / 360.0;
    static double LIMIT_BUFFER = 90;
    static double LIMIT_SPIN = 130;
    double lastError = 0, lastTime = 0;
    double integral = 0;
    static double maxIntegral = 50;
    static double kP = 0.055;
    static double kI = 0;
    static double kD = 0.067;
    static double kS = 0.22;

    Constants.Alliance alliance;
    Pose currentPose;
    ElapsedTime timer = new ElapsedTime();
    double currentTime = 0;
    Mode mode = Mode.IDLE;

    public enum Mode {
        IDLE,
        TRACKING,
        MANUAL,
    }
    public Turret(@NonNull HardwareMap hardwareMap, @NonNull Pose startPose, @NonNull Constants.Alliance alliance) {
        motor = hardwareMap.get(DcMotorEx.class, "tower");

        currentPose = startPose;
        this.alliance = alliance;

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motor.setDirection(DcMotorEx.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        timer.reset();
    }

    public void setManualPower(double power) {
        manualPower = power;
    }

    public void resetController() {
        integral = 0;
        lastError = 0;
    }

    public void update(Pose pose) {
        currentPose = pose;

        currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = dt;

        switch (mode) {

            case TRACKING:

                setPower(calculatePower(dt));
                break;

            case MANUAL:
                double currentAngle = getRawAngle();
                double safePower = manualPower;

                if (manualPower > 0) { // Indo para o limite positivo
                    if (currentAngle % 360 >= LIMIT_SPIN) {
                        safePower = 0;
                    } else if (currentAngle % 360 > LIMIT_SPIN - LIMIT_BUFFER) {
                        double distanceToLimit = LIMIT_SPIN - (currentAngle % 360);
                        safePower = manualPower * (distanceToLimit / LIMIT_BUFFER);
                    }
                }
                else if (manualPower < 0) {
                    if (currentAngle % 360 <= -LIMIT_SPIN) {
                        safePower = 0;
                    } else if (currentAngle % 360 < -LIMIT_SPIN + LIMIT_BUFFER) {
                        double distanceToLimit = (currentAngle % 360) - (-LIMIT_SPIN);
                        safePower = manualPower * (distanceToLimit / LIMIT_BUFFER);
                    }
                }

                setPower(safePower);
                break;

            case IDLE:
            default:
                setPower(0);
                break;
        }
    }
    public void setMode(Mode newMode) {
        if (this.mode != Mode.TRACKING && newMode == Mode.TRACKING) {
            resetController();
        }
        this.mode = newMode;
    }
    public Mode getMode() {
        return mode;
    }
    public Pose getTrackingPose() {

        if (alliance == Constants.Alliance.BLUE) {

            if (currentPose.y() >= 70.75) {
                return Constants.topNestBLUE;
            } else return Constants.bottomNestBLUE;

        } else {

            if (currentPose.y() >= 70.75) {
                return Constants.topNestRED;
            } else return Constants.bottomNestRED;
        }

    }

    public Pose getTurretPose() {
        double heading = currentPose.heading();

        double rotated_offsetX = OFFSET_X * Math.cos(heading) - OFFSET_Y * Math.sin(heading);
        double rotated_offsetY = OFFSET_X * Math.sin(heading) + OFFSET_Y * Math.cos(heading);

        double turret_x = currentPose.x() + rotated_offsetX;
        double turret_y = currentPose.y() + rotated_offsetY;

        return new Pose(turret_x, turret_y);
    }

    public double getRawAngle() {
        return motor.getCurrentPosition() / ticksPerDegree;
    }
    public double getAbsoluteAngle() {
        return Math.toDegrees(currentPose.heading()) + getRawAngle();
    }


    public void resetEncoder() {
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }


    public double calculatePower(double dt) {
        double error = normalizeDegrees(computeTargetAngle() - getRawAngle());

        integral += error * dt;
        integral = Range.clip(integral, -maxIntegral, maxIntegral);

        double derivative = (error - lastError) / dt;
        lastError = error;

        double pOut = kP * error;
        double iOut = kI * integral;
        double dOut = kD * derivative;

        // kS: Força mínima para vencer o atrito estático.
        // Só aplicamos se o erro for maior que 1 grau para evitar trepidação no alvo.
        double sOut = 0;
        if (Math.abs(error) > 1.0) {
            sOut = Math.signum(error) * kS;
        }

        return pOut + iOut + dOut + sOut;
    }

    private void setPower(double power) {
        motor.setPower(Range.clip(power, -maxPower, maxPower));
    }

    public double computeTargetAngle() {

        Pose turretPose = getTurretPose();
        Pose trackedPose = getTrackingPose();


        double globalDesiredAngle = Math.toDegrees(
                Math.atan2(trackedPose.y() - turretPose.y(), trackedPose.x() - turretPose.x())
        );


        double robotHeadingDeg = Math.toDegrees(currentPose.heading());

        double localDesiredAngle = globalDesiredAngle - robotHeadingDeg;

        double normalizedTarget = normalizeDegrees(localDesiredAngle);
        // Normaliza o alvo para mantê-lo entre -180 e 180
        return -Range.clip(normalizedTarget, -LIMIT_SPIN, LIMIT_SPIN);
    }

    public double clampToLimits(double angles) {
        return Range.clip(angles, -LIMIT_SPIN, LIMIT_SPIN);
    }

    public double normalizeDegrees(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

}
