package org.firstinspires.ftc.teamcode.pedro.opmodes.autonomous.tests;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;
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
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;

import com.pedropathing.utils.Timer;



@Autonomous
public class autoBlueTest extends OpMode {
    Shooter shooter;


    PoseFactory poseFactory = PoseFactory.degrees();
    Scheduler scheduler;
    private Follower follower;
    private ElapsedTime runtime = new ElapsedTime();

    Pose startingPoint = poseFactory.of(86.06466512702079,133.1605080831409,270);
    Pose collect1 = poseFactory.of(132.4805984673269,94.36056581986145,0);
    Pose shoot2 = poseFactory.of(81.96651270207855,16.393764434180113,45);
    Pose collect2 = poseFactory.of(92.53464203233256,8.706697459584301,280);
    Pose shoot3 = poseFactory.of(80.1431870669746,8.654734411085453,90);


    public Path START_TO_COLLECT1() {
        return curve(startingPoint, poseFactory.of(96.2482678983834, 103.30023094688222, 270) ,collect1).linear(startingPoint, collect1);
    }
    public Path COLLECT1_TO_SHOOT2() {
        return curve(collect1, poseFactory.of(112.70438799076214, 39.215935334872974, 0),shoot2).linear(collect1, shoot2);
    }
    public Path SHOOT2_TO_COLLECT2() {
        return line(shoot2, collect2).linear(shoot2, collect2);
    }
    public Path COLLECT2_TO_SHOOT3() {
        return curve(collect2, poseFactory.of(85.93418013856814, 34.46535796766743, 280),shoot3).linear(collect2, shoot3);
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startingPoint);

        shooter = new Shooter(hardwareMap);

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
                shooter.shootPP(),
                follow(follower, START_TO_COLLECT1()),
                //intake
                follow(follower, COLLECT1_TO_SHOOT2()),
                shooter.shootPP()




        );
    }

}