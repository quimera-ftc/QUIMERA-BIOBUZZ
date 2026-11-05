package org.firstinspires.ftc.teamcode.pedro.opmodes.tests;

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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Turret;

import java.util.List;

@Autonomous
public class downTestBlue extends OpMode {

    Shooter shooter;
    Intake intake;
    Turret turret;

    PoseFactory poseFactory =  PoseFactory.degrees();
    private Follower follower;

    List<LynxModule> allHubs;


    Pose start = poseFactory.of(91.02,37.98,270);
    Pose collect = poseFactory.of(93.86,16.61,270);
    Pose  shoot = poseFactory.of(78.63,9.73,90);
    Pose fim = poseFactory.of(129.45,17.79,90);

    public Path START_TO_COLLECT(){
        return line(start,collect).linear(start,collect);
    }
    public Path COLLECT_TO_SHOOT(){
        return line(collect,shoot).linear(collect,shoot);
    }

    public Path SHOOT_TO_FIM() {
        return line(shoot, fim).linear(shoot, fim);
    }


    public void init(){
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap, start, Constants.Alliance.BLUE);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void start(){
        shooter.powerUp();
        Scheduler.schedule(routine());
    }


    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public Command routine() {
        return parallel(
                sequential(
                waitMs(2000),
                shooter.shootPP(),
                follow(follower, START_TO_COLLECT()),
                parallel(
                        follow(follower, COLLECT_TO_SHOOT()),
                        intake.intakePP()
                ),
                instant(() -> intake.stop()),
                follow(follower,SHOOT_TO_FIM()),
                shooter.shootPP()

                ), turret.autoaim(follower::pose));
    }

}


