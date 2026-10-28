package org.firstinspires.ftc.teamcode.pedro.opmodes.autonomous.tests;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
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

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;




@Autonomous
public class autoBlueTest extends OpMode {
    Shooter shooter;
    Intake intake;


    PoseFactory poseFactory = PoseFactory.degrees();
    Scheduler scheduler;
    private Follower follower;

    Pose startingPoint = poseFactory.of(86.06466512702079,133.1605080831409,270);
    Pose collect1 = poseFactory.of(130.4805984673269,94.36056581986145,0);
    Pose shoot2 = poseFactory.of(81.96651270207855,16.393764434180113,45);
    Pose collect2 = poseFactory.of(92.53464203233256,8.706697459584301,280);
    Pose shoot3 = poseFactory.of(80.1431870669746,8.654734411085453,90);
    Pose park = poseFactory.of(132.4109489051095, 19.869388686131376, 0);


    public Path START_TO_COLLECT1() {
        return curve(startingPoint, poseFactory.of(96.2482678983834, 103.30023094688222, 270) ,collect1).linear(startingPoint, collect1);
    }
    public Path COLLECT1_TO_SHOOT2() {
        return curve(collect1, poseFactory.of(106.92044638492273, 44.17359956844962, 0),shoot2).linear(collect1, shoot2);
    }
    public Path SHOOT2_TO_COLLECT2() {
        return line(shoot2, collect2).linear(shoot2, collect2);
    }
    public Path COLLECT2_TO_SHOOT3() {
        return curve(collect2, poseFactory.of(85.93418013856814, 34.46535796766743, 280),shoot3).linear(collect2, shoot3);
    }
    public Path SHOOT3_TO_PARK() {
        return curve(shoot3, poseFactory.of(88.94791666666669, 21.488425925925924, 90),park).linear(shoot3, park);
    }


    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startingPoint);

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

    }

    @Override
    public void start() {
        shooter.powerUp();
        Scheduler.schedule(routine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

    }

    public Command routine() {
        return sequential(
                waitMs(2000),
                shooter.shootPP(),
                parallel(
                        follow(follower, START_TO_COLLECT1()),
                        intake.intakePP()
                ),
                instant(() -> intake.stop()),
                follow(follower, COLLECT1_TO_SHOOT2()),
                shooter.shootPP(),
                parallel(
                        follow(follower, SHOOT2_TO_COLLECT2()),
                        intake.intakePP()
                ),
                instant(() -> intake.stop()),
                follow(follower, COLLECT2_TO_SHOOT3()),
                shooter.shootPP(),
                follow(follower, SHOOT3_TO_PARK())
        );
    }
}