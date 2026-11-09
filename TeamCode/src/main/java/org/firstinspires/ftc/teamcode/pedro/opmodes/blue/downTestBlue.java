package org.firstinspires.ftc.teamcode.pedro.opmodes.blue;

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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Turret;

@Autonomous
public class downTestBlue extends OpMode {

    Shooter shooter;
    Intake intake;
    Turret turret;

    PoseFactory poseFactory =  PoseFactory.degrees();



    private Follower follower;

    private final Pose torre = poseFactory.of(78.5051, 8.889, 90);
    private final Pose start =  poseFactory.of(91.0176, 37.9776, 270);
    private final Pose collectStart = poseFactory.of(93.86, 16.61, 270);
    private  final Pose collect = poseFactory.of(94.04,26.23,270);
    private final  Pose turneStart = poseFactory .of(94.04,26.23,270);
    private final Pose turne = poseFactory.of(91.0176,37.9776,270);

    private final Pose shootStart = poseFactory.of(93.8607, 16.6075, 270);
   private final Pose  shoot = poseFactory.of(78.63,9.73,90);
    private final Pose fimStart = poseFactory.of(78.6305, 9.7349, 90);
   private final Pose fim = poseFactory.of(129.45,17.79,90);

    public Path START(){
        return line(torre,start).linear(torre,start);
    }
    public Path COLLECT(){
        return line(collectStart,collect).linear(collectStart,collect);
    }
    public  Path TURNE(){
        return line(turneStart,turne).linear(turneStart,turne);
    }

    public Path SHOOT() {
        return line(shootStart, shoot).linear(shootStart, shoot);
    }
    public Path FIM(){
        return line(fimStart,fim).linear(fimStart, fim);
    }



    public void init(){
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(torre);

        shooter = new Shooter(hardwareMap, intake);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap, torre, Constants.Alliance.BLUE);




    }

    public void start(){
        shooter.powerUp();
        Scheduler.schedule(routine());
    }


    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        intake.update();
    }

    public Command routine() {
        return parallel(
                sequential(
                        waitMs(5000),
                shooter.shootPP(),
                follow(follower, START()),
                parallel(
                        follow(follower, COLLECT()),
                        intake.intakePP()
                ),
                instant(() -> intake.stop()),
                follow(follower,TURNE()),
                follow(follower,SHOOT()),
                        shooter.shootPP(),
                        follow(follower, FIM())

                ), turret.autoaim(follower::pose));
    }

}


