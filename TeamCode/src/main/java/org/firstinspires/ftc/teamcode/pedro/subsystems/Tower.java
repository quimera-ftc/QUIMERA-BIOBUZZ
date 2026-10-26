package org.firstinspires.ftc.teamcode.pedro.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import androidx.annotation.NonNull;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Tower {
    double gearRatio = 50.0 / 15.0;
    double ticksPerRev, ticksPerDegree;

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

    private final DcMotor tower;
    private final double towerPower = 0.7, maxSpin = 150;
    private final double[] offsets = new double[] {0, 3};
    private TowerState state = TowerState.IDLE, lastState = TowerState.IDLE;
    private final Pose topNestBLUE = new Pose(84, 84);
    private final Pose bottomNestBLUE = new Pose(84, 59);
    private final Pose topNestRED = new Pose(60, 84);
    private final Pose bottomNestRED = new Pose(60, 59);
    private Pose trackedPose = topNestBLUE, currentPose;

    private Direction rotatingTo;

    public Tower(HardwareMap hardwareMap) {
        tower = hardwareMap.get(DcMotor.class, "tower");



        tower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        tower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        ticksPerRev = tower.getMotorType().getTicksPerRev();
        if (ticksPerRev == 0) ticksPerRev = 28;
        ticksPerDegree = (ticksPerRev * gearRatio) / 360.0;
    }

    private void rotateInternal(@NonNull Direction direction) {

        if (state == TowerState.LOCKED) {
            return;
        }

        if (dislocation() >= maxSpin && direction == Direction.RIGHT ||
                dislocation() <= -maxSpin && direction == Direction.LEFT) {
            stopInternal();
            return;
        }

        tower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (direction == Direction.RIGHT) {

            tower.setPower(towerPower);

        } else {
            tower.setPower(-towerPower);
        }

        rotatingTo = direction;
        state = TowerState.ROTATING;
    }

    public Command rotate(@NonNull Direction direction) {
        return instant(() -> rotateInternal(direction));
    }

    private void lockInternal() {
        lastState = state;
        state = TowerState.LOCKED;

        tower.setTargetPosition(tower.getCurrentPosition());
        tower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tower.setPower(1);
    }

    public Command lock() {
        return instant(this::lockInternal);
    }

    private void unlockInternal() {
        state = lastState;
        tower.setPower(0);
        tower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Command unlock() {
        return instant(this::unlockInternal);
    }

    private void updateInternal(@NonNull Pose pose) {
        currentPose = pose;
        fix();

    }

    public Command update(@NonNull Pose pose) {
        return instant(() -> updateInternal(pose));
    }


    private void stopInternal() {
        state = TowerState.IDLE;
        tower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tower.setPower(0);
    }

    public Command stop() {
        return instant(this::stopInternal);
    }

    private void adjust(double degrees) {
        if (state == TowerState.LOCKED || state != TowerState.TRACKING) {
            return;
        }
        int target = Math.toIntExact(tower.getCurrentPosition() + Math.round(ticksPerDegree * degrees));
        tower.setTargetPosition(target);
        tower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tower.setPower(towerPower);
    }

    private void enforceLimits() {

        double currentDislocation = dislocation();

        if (state == TowerState.ROTATING || state == TowerState.TRACKING) {

            if (currentDislocation >= maxSpin && rotatingTo == Direction.RIGHT ||
                    currentDislocation <= -maxSpin && rotatingTo == Direction.LEFT)  {
                stopInternal();
            }

        }
    }

    private void fix() {

        enforceLimits();

        if (currentPose == null) return;

        if (state == TowerState.TRACKING) {
            double current = dislocation();
            double desired = Range.clip(current - calculate_correction(), -maxSpin, maxSpin);
            adjust(desired - current);
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

    private void trackInternal(@NonNull TrackingPose pose) {

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

    public Command track(@NonNull TrackingPose trackingPose) {
        return instant(() -> trackInternal(trackingPose));
    }

    public double dislocation() {
        int ticks = tower.getCurrentPosition();
        return ticks / ticksPerDegree;
    }

    private double calculate_correction() {

        double headingRad = currentPose.heading();

        double headingTurret = Math.toDegrees(headingRad) - dislocation();

        double turret_x = currentPose.x() + (offsets[0] * Math.cos(headingRad) - offsets[1] * Math.sin(headingRad));
        double turret_y = currentPose.y() + (offsets[0] * Math.sin(headingRad) + offsets[1] * Math.cos(headingRad));

        double deltaX, deltaY, theta, correction;

        deltaX = trackedPose.x() - turret_x;
        deltaY = trackedPose.y() - turret_y;

        theta = Math.toDegrees(Math.atan2(deltaY, deltaX));
        correction = theta - headingTurret;

        return ((correction % 360) + 540) % 360 - 180;

    }
}
