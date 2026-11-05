package org.firstinspires.ftc.teamcode.pedro.math;

import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.LIMIT_SPIN;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.OFFSET_X;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.OFFSET_Y;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.TOLERANCE;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.kD;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.kI;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.kP;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.kS;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.Range;


public class TurretController {
    public static Pose getTurretPose(Pose currentPose) {
        double heading = currentPose.heading();

        double rotated_offsetX = OFFSET_X * Math.cos(heading) - OFFSET_Y * Math.sin(heading);
        double rotated_offsetY = OFFSET_X * Math.sin(heading) + OFFSET_Y * Math.cos(heading);

        double turret_x = currentPose.x() + rotated_offsetX;
        double turret_y = currentPose.y() + rotated_offsetY;

        return new Pose(turret_x, turret_y);
    }
    public static double calculatePower(double integral, double error, double filteredDerivative) {

        double pOut = kP * error;
        double iOut = kI * integral;
        double dOut = kD * filteredDerivative;


        double errorMagnitude = Math.abs(error);

        double kSFraction = Math.min(
                1.0,
                errorMagnitude / (2.0 * TOLERANCE)
        );

        double sOut = Math.signum(error) * kS * kSFraction;

        return pOut + iOut + dOut + sOut;
    }
    public static double computeTargetAngle(Pose currentPose, Pose trackedPose) {

        Pose turretPose = TurretController.getTurretPose(currentPose);

        double globalDesiredAngle = Math.toDegrees(
                Math.atan2(trackedPose.y() - turretPose.y(), trackedPose.x() - turretPose.x())
        );


        double robotHeadingDeg = Math.toDegrees(currentPose.heading());

        double localDesiredAngle = globalDesiredAngle - robotHeadingDeg;

        double normalizedTarget = normalizeDegrees(localDesiredAngle);
        return -Range.clip(normalizedTarget, -LIMIT_SPIN, LIMIT_SPIN);
    }
    public static double normalizeDegrees(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public static double clampToLimits(double angles) {
        return Range.clip(angles, -LIMIT_SPIN, LIMIT_SPIN);
    }
}
