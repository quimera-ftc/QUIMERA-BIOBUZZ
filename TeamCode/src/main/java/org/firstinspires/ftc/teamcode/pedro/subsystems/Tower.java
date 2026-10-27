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


    public final DcMotor motor;
    private final double towerPower = 0.7, maxSpin = 150;
    private final double[] offsets = new double[] {0, 3};
    private TowerState state = TowerState.IDLE, lastState = TowerState.IDLE;
    private final Pose topNestBLUE = new Pose(84, 84);
    private final Pose bottomNestBLUE = new Pose(84, 59);
    private final Pose topNestRED = new Pose(60, 84);
    private final Pose bottomNestRED = new Pose(60, 59);
    private Pose trackedPose = topNestBLUE, currentPose;

    private int rotatingTo;

    public Tower(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "tower");



        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        ticksPerRev = motor.getMotorType().getTicksPerRev();
        if (ticksPerRev == 0) ticksPerRev = 28;
        ticksPerDegree = (ticksPerRev * gearRatio) / 360.0;
    }

    public void rotate(int direction) {

        if (state == TowerState.LOCKED) {
            return;
        }

        if (dislocation() >= maxSpin && direction == 1 ||
                dislocation() <= -maxSpin && direction == 0) {
            stop();
            return;
        }

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(towerPower * direction);

        rotatingTo = direction;
        state = TowerState.ROTATING;
    }

    public void lock() {
        lastState = state;
        state = TowerState.LOCKED;

        motor.setTargetPosition(motor.getCurrentPosition());
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
    }

    public void unlock() {
        state = lastState;
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update(@NonNull Pose pose) {
        currentPose = pose;
        fix();

    }


    public void stop() {
        state = TowerState.IDLE;
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }

    private void adjust(double degrees) {
        if (state == TowerState.LOCKED || state != TowerState.TRACKING) {
            return;
        }
        int target = Math.toIntExact(motor.getCurrentPosition() + Math.round(ticksPerDegree * degrees));
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(towerPower);
    }

    private void enforceLimits() {

        double currentDislocation = dislocation();

        if (state == TowerState.ROTATING || state == TowerState.TRACKING) {

            if (currentDislocation >= maxSpin && rotatingTo == 1 ||
                    currentDislocation <= -maxSpin && rotatingTo == -1)  {
                stop();
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
        int ticks = motor.getCurrentPosition();
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
