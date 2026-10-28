package org.firstinspires.ftc.teamcode.pedro.opmodes.autonomous.tests;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class autoPathTesting extends OpMode {


    PoseFactory poseFactory = PoseFactory.degrees();
    private Follower follower;

    Pose startingPoint = poseFactory.of(83,130,270);
    Pose followingPoint = poseFactory.of(31, 102, 90);

    public Path TEST() {
        return line(startingPoint, followingPoint).linear(startingPoint, followingPoint);
    }


    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startingPoint);

    }

    public void loop() {
        follower.update();
        Scheduler.execute();

    }

    public void start() {
        Scheduler.schedule(routine());

    }

    public Command routine() {
        return sequential(
                follow(follower, TEST())

        );

    }

}
