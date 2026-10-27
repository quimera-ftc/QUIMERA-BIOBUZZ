package org.firstinspires.ftc.teamcode.pedro.opmodes.autonomous.tests;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;

@Autonomous

public class downTestBlue extends OpMode {

    Shooter shooter;
    Intake intake;

    PoseFactory poseFactory = PoseFactory.degrees();

    private Follower follower;


    Pose start = poseFactory.of(47.71,111.39,270);
    Pose collect = poseFactory.of(47.08,128.10,90);
    Pose turn = poseFactory.of(47.93,111.90,90);
    Pose  shoot = poseFactory.of(47.71,111.39,270);
    Pose front = poseFactory.of(58.67,10.22,180);
    Pose collect2 = poseFactory.of(8.912,)




}
