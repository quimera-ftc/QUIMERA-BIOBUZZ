package org.firstinspires.ftc.teamcode.pedro.subsystems;

import androidx.annotation.NonNull;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    int toleranceTicks = 1;
    private final double[] offsets = new double[] {0, -1,772};
    private TowerState state = TowerState.IDLE, lastState = TowerState.IDLE;
    private final Pose topNestBLUE = new Pose(84, 84);
    private final Pose bottomNestBLUE = new Pose(84, 59);
    private final Pose topNestRED = new Pose(60, 84);
    private final Pose bottomNestRED = new Pose(60, 59);
    private Pose trackedPose = topNestBLUE, currentPose;
    private final Telemetry telemetry;
    private int rotatingTo;
    private double lastCorrection;

    public Tower(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        tower = hardwareMap.get(DcMotorEx.class, "tower");

        this.telemetry = telemetry;

        tower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tower.setDirection(DcMotor.Direction.REVERSE);
        tower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tower.setTargetPositionTolerance(2);
    }

    public Tower(@NonNull HardwareMap hardwareMap) {
        tower = hardwareMap.get(DcMotorEx.class, "tower");

        telemetry = null;

        tower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tower.setDirection(DcMotor.Direction.REVERSE);
        tower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tower.setTargetPositionTolerance(2);
    }

    public void rotate(int direction) {

        if (state == TowerState.LOCKED) {
            return;
        }

        if (dislocation() >= maxSpin && direction == 1 ||
                dislocation() <= -maxSpin && direction == -1) {
            stop();
            return;
        }

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        tower.setPower(towerPower * direction);

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
    }


    public void update(@NonNull Pose pose) {
        currentPose = pose;
        enforceLimits();
        fix();
    }

    public void printTelemetry() {
        if (telemetry != null) {
            telemetry.addData("estado torre", state);
            telemetry.addData("ang. torre", dislocation());
            telemetry.addData("correção torre", lastCorrection);
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

    private void adjust(double degrees) {
        if (state != TowerState.TRACKING) {
            return;
        }
//
//        if (degrees <= 2.5) {
//            return;
//        }

        double clipped = Range.clip(degrees, -maxSpin, maxSpin);
        int target = (int) Math.round(ticksPerDegree * clipped);

        if (tower.getMode() == DcMotor.RunMode.RUN_TO_POSITION &&
                Math.abs(target - tower.getTargetPosition()) <= toleranceTicks) {
            return;
        }

        tower.setTargetPosition(target);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tower.setPower(towerPower);
    }

    private void enforceLimits() {
        double currentDislocation = dislocation();

        if (state == TowerState.ROTATING) {

            if (currentDislocation >= maxSpin && rotatingTo == 1 ||
                    currentDislocation <= -maxSpin && rotatingTo == -1)  {
                stop();
            }
        }
    }

    private void fix() {

        if (currentPose == null) return;

        if (state == TowerState.TRACKING) {
            double current = dislocation();
            lastCorrection = calculateCorrection();
            double desired = current + lastCorrection;

            double best = Double.NaN;
            for (double candidate : new double[] {desired, desired - 360, desired + 360}) {
                if (Math.abs(candidate) <= maxSpin &&
                        (Double.isNaN(best) || Math.abs(candidate - current) < Math.abs(best - current))) {
                    best = candidate;
                }
            }

            adjust(Double.isNaN(best) ? desired : best);

            if (telemetry != null) {
                telemetry.addData("turret dislocation", current);
                telemetry.addData("desired", desired);
                telemetry.addData("best", !Double.isNaN(best) ? best : "x.x");
            }
        }
    }

    public boolean isBusy() {
        return state != TowerState.IDLE;
    }
    public boolean isTracking() {
        return state == TowerState.TRACKING;
    }
    public boolean isLocked() {
        return state == TowerState.LOCKED;
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

        state = TowerState.TRACKING;
    }
    public double dislocation() {
        int ticks = tower.getCurrentPosition();
        return ticks / ticksPerDegree;
    }
    public double calculateCorrection() {

        double headingRad = currentPose.heading();

        double headingTurret = Math.toDegrees(headingRad) + dislocation();

        double turret_x = currentPose.x() + (offsets[0] * Math.cos(headingRad) - offsets[1] * Math.sin(headingRad));
        double turret_y = currentPose.y() + (offsets[0] * Math.sin(headingRad) + offsets[1] * Math.cos(headingRad));

        double deltaX, deltaY, theta, correction;

        deltaX = trackedPose.x() - turret_x;
        deltaY = trackedPose.y() - turret_y;

        theta = Math.toDegrees(Math.atan2(deltaY, deltaX));
        correction = normalize(theta - headingTurret);

        return correction;
    }
    private static double normalize(double degrees) {
        degrees %= 360;
        if (degrees > 180) degrees -= 360;
        if (degrees <= -180) degrees += 360;
        return degrees;
    }
}