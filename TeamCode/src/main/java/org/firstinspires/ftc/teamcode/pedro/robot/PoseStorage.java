package org.firstinspires.ftc.teamcode.pedro.robot;

import com.pedropathing.math.Pose;

public class PoseStorage {
    public static Pose currentPose = new Pose(132.12685185185188, 51.289675925925924, Math.toRadians(270));

    private static Pose autonomousEnd;

    private PoseStorage() {
        //aa
    }

    public static void save(Pose pose) {
        autonomousEnd = pose;
    }

    public static Pose getPose() {
        return autonomousEnd;
    }

    public static boolean hasAutonomousEnd() {
        return autonomousEnd != null;
    }

    public static void clear() {
        autonomousEnd = null;
    }
}
