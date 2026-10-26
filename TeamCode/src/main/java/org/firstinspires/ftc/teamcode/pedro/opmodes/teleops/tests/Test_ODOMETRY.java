package org.firstinspires.ftc.teamcode.pedro.opmodes.teleops.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;

@TeleOp
public class Test_ODOMETRY extends OpMode {

    DriveTrain driveTrain;
    Follower follower;


    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
        follower.setPose(new Pose(133.81725311873922,132.57458675808917, Math.toRadians(180)));
    }

    @Override
    public void loop() {
        follower.update();

        Pose currentPose = follower.pose();

        telemetry.addData("x", currentPose.x());
        telemetry.addData("y", currentPose.y());
        telemetry.addData("heading", Math.toDegrees(currentPose.heading()));


        driveTrain.driveStick();
        telemetry.update();
    }

}
