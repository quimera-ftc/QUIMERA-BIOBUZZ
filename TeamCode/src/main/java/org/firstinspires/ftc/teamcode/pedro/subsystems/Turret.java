package org.firstinspires.ftc.teamcode.pedro.subsystems;

import androidx.annotation.NonNull;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Turret {

    // --- mecanico ---
    DcMotorEx motor;
    static double OFFSET_Y = 0.0;
    static double OFFSET_X = -1.772;
    double gearRatio = 50.0 / 15.0;
    double ticksPerRev = 28.0 * (76.0 / 21.0) * gearRatio; // HD Hex = 28 ticks/volta; redução "4:1" é na real 76/21 (~3.62:1)
    double ticksPerDegree = ticksPerRev / 360.0;

    Constants.Alliance alliance;
    Pose currentPose;

    public enum Mode {
        OFF,
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

        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public Pose getTrackingdPose() {

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

    public Pose getAbsolutePose() {
        double turret_x = OFFSET_X * Math.cos(currentPose.heading()) - OFFSET_Y * Math.sin(currentPose.heading());
        double turret_y = OFFSET_X * Math.sin(currentPose.heading()) + OFFSET_Y * Math.cos(currentPose.heading());

        return new Pose(turret_x, turret_y);
    }

    public double getRawAngle() {
        return motor.getCurrentPosition() / ticksPerDegree;
    }

    public double getAbsoluteAngle() {
        return Math.toDegrees(currentPose.heading()) + getRawAngle();
    }

    public void setAbsoluteTarget() {

    }

    public double calculateDesiredAbsoluteAngle() {

        Pose turretPose = getAbsolutePose();
        Pose trackedPose = getTrackingdPose();

        double desired = Math.toDegrees(
                Math.atan2(trackedPose.y() - turretPose.y(), trackedPose.x() - turretPose.x())
        );

        return normalizeDegrees(desired);
    }

    private double normalizeDegrees(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

}
