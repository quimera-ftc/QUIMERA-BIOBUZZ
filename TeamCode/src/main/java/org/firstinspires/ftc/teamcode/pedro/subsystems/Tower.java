package org.firstinspires.ftc.teamcode.pedro.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import androidx.annotation.NonNull;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    double towerPower = 0.7, maxSpin = 120;
    private final double[] offsets = new double[] {0, 3};
    private TowerState state = TowerState.IDLE;
    private final Pose topNestBLUE = new Pose(84, 84);
    private final Pose bottomNestBLUE = new Pose(84, 59);
    private final Pose topNestRED = new Pose(59, 84);
    private final Pose bottomNestRED = new Pose(59, 59);
    private Pose trackedPose = topNestBLUE, current_pose;
    Telemetry telemetry;


    public Tower(HardwareMap hardwareMap, Telemetry telemetry) {
        tower = hardwareMap.get(DcMotor.class, "tower");

        this.telemetry = telemetry;

        tower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        tower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ticksPerRev = tower.getMotorType().getTicksPerRev();
        ticksPerDegree = (ticksPerRev * gearRatio) / 360.0;
    }

    private void rotateInternal(@NonNull Direction direction) {

        if ((dislocation() >= maxSpin && direction == Direction.RIGHT) ||
                (dislocation() <= -maxSpin && direction == Direction.LEFT)) {
            stop();
            return;
        }

        tower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (direction == Direction.RIGHT) {

            tower.setPower(towerPower);

        } else {
            tower.setPower(-towerPower);
        }

        state = TowerState.ROTATING;
    }

    public Command rotate(@NonNull Direction direction) {
        return instant(() -> rotateInternal(direction));
    }

    private void lockInternal() {
        state = TowerState.LOCKED;
        tower.setTargetPosition(tower.getCurrentPosition());
        tower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tower.setPower(1);
    }

    public Command lock() {
        return instant(this::lockInternal);
    }

    private void unlockInternal() {
        state = TowerState.IDLE;
        tower.setPower(0);
        tower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Command unlock() {
        return instant(this::unlockInternal);
    }

    private void updateInternal(@NonNull Pose pose) {
        current_pose = pose;
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

    private void adjustInternal(double degrees) {
        if (state == TowerState.LOCKED) {
            return;
        }

        int target = tower.getCurrentPosition() + (int) (ticksPerDegree * degrees);

        tower.setTargetPosition(target);
        tower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tower.setPower(towerPower);
    }

    public Command adjust(double degrees) {
        return instant(() -> adjustInternal(degrees));
    }

    private void fix() {
        if (state != TowerState.TRACKING) return;
        double current = dislocation();
        double desired = Range.clip(current - calculate_correction(), -maxSpin, maxSpin);
        adjustInternal(desired - current);
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

        double headingRad = current_pose.heading();

        double headingTurret = Math.toDegrees(headingRad) - dislocation();

        double turret_x = current_pose.x() + (offsets[0] * Math.cos(headingRad) - offsets[1] * Math.sin(headingRad));
        double turret_y = current_pose.y() + (offsets[0] * Math.sin(headingRad) + offsets[1] * Math.cos(headingRad));

        double deltaX, deltaY, theta, correction;

        deltaX = trackedPose.x() - turret_x;
        deltaY = trackedPose.y() - turret_y;

        theta = Math.toDegrees(Math.atan2(deltaY, deltaX));
        correction = theta - headingTurret;

        return ((correction % 360) + 540) % 360 - 180;

    }
}
