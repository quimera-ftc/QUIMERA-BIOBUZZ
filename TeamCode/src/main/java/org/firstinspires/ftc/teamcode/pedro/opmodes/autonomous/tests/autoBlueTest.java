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
import com.pedropathing.api.Paths.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.utils.Timer;



@Autonomous
public class autoBlueTest extends OpMode {

    PoseFactory p = PoseFactory.degrees();
    Scheduler scheduler;
    private Follower follower;
    private ElapsedTime runtime = new ElapsedTime();

    Pose startPose = p.of(82.72808988764045,7.367415730337147,90);
    Pose walkPose = p.of(82.72808988764045,27.902528089887642,90);
    Pose finalPose = p.of(103.97752808988766,27.902528089887642,180);


    public Path path1() {
        return line(startPose, walkPose);
    }
    public Path path2() {
        return line(walkPose, finalPose);
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower.setPose(startPose);

    }

    @Override
    public void start() {
        Scheduler.schedule(routine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }

    public Command routine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2())
        );
    }

}