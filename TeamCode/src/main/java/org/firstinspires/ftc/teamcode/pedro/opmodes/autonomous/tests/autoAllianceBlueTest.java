package org.firstinspires.ftc.teamcode.pedro.opmodes.autonomous.tests;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Agropecuaria;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;

public class autoAllianceBlueTest extends OpMode {

    Shooter shooter;
    Intake intake;
    Agropecuaria agro;


    PoseFactory poseFactory = PoseFactory.degrees();
    private Follower follower;


    Pose startingPoint = poseFactory.of(86.06466512702079,134.1431469720298,0);
    Pose collect1 = poseFactory.of(132.97337962962962, 131.97106481481478, 0);
    Pose shoot2 = poseFactory.of(105.32466512702079,129.2331469720298,0);
    Pose collect2 = poseFactory.of(132.8888888888889, 94.93055555555556, 0);
    Pose shoot3 = poseFactory.of(105.32466512702079, 129.2331469720298, 0);
    Pose park = poseFactory.of(132.12685185185188, 51.289675925925924, 270);


    public Path START_TO_COLLECT1() {
        return line(startingPoint, collect1).linear(startingPoint, collect1);
    }

    public Path COLLECT1_TO_SHOOT2() {
        return line(collect1, shoot2).linear(collect1, shoot2);
    }

    public Path SHOOT2_TO_COLLECT2() {
        return curve(shoot2, poseFactory.of(99.73, 95.99, 0),collect2).linear(shoot2, collect2);
    }

    public Path COLLECT2_TO_SHOOT3() {
        return curve(collect2, poseFactory.of(99.73, 95.99, 0),shoot3).linear(collect2, shoot3);
    }

    public Path SHOOT3_TO_PARK() {
        return line(shoot3, park).linear(shoot3, park);
    }


    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startingPoint);

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        agro = new Agropecuaria(hardwareMap);

    }

    public void start() {
        shooter.powerUp();
        Scheduler.schedule(routine());

    }

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
                intake.intakestopPP(),
                follow(follower, COLLECT1_TO_SHOOT2()),
                waitMs(12000),
                shooter.shootPP(),
                parallel(
                        follow(follower, SHOOT2_TO_COLLECT2()),
                        intake.intakePP()
                ),
                intake.intakestopPP(),
                follow(follower, COLLECT2_TO_SHOOT3()),
                shooter.shootPP(),
                follow(follower, SHOOT3_TO_PARK())
        );

    }

}
