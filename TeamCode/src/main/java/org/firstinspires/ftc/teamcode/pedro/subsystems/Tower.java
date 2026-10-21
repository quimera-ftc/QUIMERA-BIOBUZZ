package org.firstinspires.ftc.teamcode.pedro.subsystems;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Tower {
    double gearRatio = 50.0 / 15.0;
    double ticksPerRev, ticksPerDegree;

    private enum TowerState {
        IDLE,
        TRACKING,
        LOCKED,
        ROTATING
    }

    public enum TrackingPoses {
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
    double towerPower = 0.5, maxSpin = 120;
    private final double[] offsets = new double[] {0,3};
    private TowerState state = TowerState.IDLE;
    private final Pose topNestBLUE = new Pose(84,84);
    private final Pose bottomNestBLUE = new Pose(84,59);
    private final Pose topNestRED = new Pose(59,84);
    private final Pose bottomNestRED = new Pose(59,59);
    private int lockedTicks = 0;
    private Pose trackedPose, current_pose;


    public Tower(@NonNull HardwareMap hardwareMap) {
        tower = hardwareMap.get(DcMotor.class, "tower");

        tower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticksPerRev = tower.getMotorType().getTicksPerRev();
        ticksPerDegree = (ticksPerRev * gearRatio) / 360.0;
    }

    public void rotate(Direction direction) {

        if (Math.abs(dislocation()) >= maxSpin) {
            return;
        }

        if (direction == Direction.RIGHT) {
            tower.setPower(towerPower);
        } else {
            tower.setPower(-towerPower);
        }

        state = TowerState.ROTATING;
    }

    public void lock() {
        lockedTicks = tower.getCurrentPosition();
        state = TowerState.LOCKED;

        tower.setTargetPosition(lockedTicks);
        tower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tower.setPower(1);
    }

    public void unlock() {
        state = TowerState.IDLE;
        tower.setPower(0);
        tower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update(@NonNull Pose pose) {
        current_pose = pose;
        fix();
    }

    public void stop() {
        state = TowerState.IDLE;
        tower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tower.setPower(0);
    }

    public void adjust(double degrees) {
        if (state == TowerState.LOCKED) {
            return;
        }

        int target = tower.getCurrentPosition() + (int) (ticksPerDegree * degrees);

        tower.setTargetPosition(target);
        tower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tower.setPower(1);
    }

    public void fix() {

        switch (state) {

            case IDLE:


            case LOCKED:


            case ROTATING:
                if (Math.abs(dislocation()) >= maxSpin) {
                    state = TowerState.IDLE;
                }

            case TRACKING:

                double correction = calculate_correction();

                if (dislocation() + correction > maxSpin) {
                    correction = maxSpin;
                }
                adjust(correction);
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

    public void track(TrackingPoses pose) {

        if (state == TowerState.LOCKED) {
            return;
        }

        switch (pose) {
            case TOP_GOAL_BLUE:
                trackedPose = topNestBLUE;

            case BOTTOM_GOAL_BLUE:
                trackedPose = bottomNestBLUE;

            case TOP_GOAL_RED:
                trackedPose = topNestRED;

            case BOTTOM_GOAL_RED:
                trackedPose = bottomNestRED;
        }
        state = TowerState.TRACKING;
    }

    public double dislocation() {
        int ticks = tower.getCurrentPosition();
        return ticks / ticksPerDegree;
    }

    private double calculate_correction() {

        double headingTurret = current_pose.heading() - dislocation();

        double turret_x = current_pose.x() + (
                offsets[0] * Math.cos(current_pose.heading())
                        - offsets[1] * Math.sin(current_pose.heading())
        );
        double turret_y = current_pose.y() + (
                offsets[0] * Math.sin(current_pose.heading())
                        + offsets[1] * Math.cos(current_pose.heading())
        );

        double deltaX, deltaY, theta, correction;

        deltaX = trackedPose.x() - turret_x;
        deltaY = trackedPose.y() - turret_y;

        theta = (Math.atan2(deltaY, deltaX) * 180) * Math.PI;
        correction = Math.abs(theta - headingTurret);

        if (headingTurret < theta) {
            correction = -correction;
        }

        return Math.round(correction);

    }
}
