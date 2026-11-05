package org.firstinspires.ftc.teamcode.pedro.opmodes.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;

public class TestTeleopBase extends OpMode {
    DriveTrain driveTrain;
    Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);

        follower.setPose(PoseStorage.currentPose);
    }

    @Override
    public void loop() {
        follower.update();

        Pose currentPose = follower.pose();

        telemetry.addData("x", currentPose.x());
        telemetry.addData("y", currentPose.y());
        telemetry.addData("heading", currentPose.heading());

        driveTrain.driveStick();
        telemetry.update();
    }
}
