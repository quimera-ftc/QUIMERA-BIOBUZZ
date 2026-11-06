package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class FieldPose {
    private static PoseFactory p = PoseFactory.degrees();
    public static final Pose topNestBLUE = p.of(84, 84,0);
    public static final Pose bottomNestBLUE = p.of(84, 57.5,0);
    public static final Pose topNestRED = p.of(57.5, 84,0);
    public static final Pose bottomNestRED = p.of(57.5, 57.5, 0);
    public static final Pose topParkBLUE = p.of(132.12685185185188, 51.289675925925924, 270);
    public static final Pose bottomParkBLUE = p.of(129.45, 17.78, 90);
}
