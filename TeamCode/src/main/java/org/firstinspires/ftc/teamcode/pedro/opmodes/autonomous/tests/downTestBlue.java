package org.firstinspires.ftc.teamcode.pedro.opmodes.autonomous.tests;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;

@Autonomous

public class downTestBlue extends OpMode {

    Shooter shooter;
    Intake intake;

    PoseFactory poseFactory = PoseFactory.degrees();
    Scheduler scgedular;

    private Follower follower;


    Pose start = poseFactory.of(47.71,111.39,270);
    Pose collect = poseFactory.of(47.08,128.10,90);
    Pose turn = poseFactory.of(47.93,111.90,90);
    Pose  shoot = poseFactory.of(47.71,111.39,270);
    Pose front = poseFactory.of(58.67,10.22,180);
    Pose collect2 = poseFactory.of(8.912,7.02,180);
    Pose turn2 = poseFactory.of(59.07,9.96,180);
    Pose  diagonal  = poseFactory.of(26.32,47.38,180);
    Pose collect3 = poseFactory.of(12.06,47.23,180);
    Pose shoot2 = poseFactory.of(8.912,7.02,180);

    public Path START_TO_COLLECT(){
        return curve(start, poseFactory.of(53.16,80.15,90),collect).linear(start,collect);
    }
    public Path COLLEct_TO_TURN(){
        return line(collect,turn).linear(collect,turn);
    }
    public  Path TURN_TO_SHOOT(){
        return line(turn,shoot).linear(turn,shoot);
    }
    public Path SHOOT_TO_FRONT(){
        return line(shoot,front).linear(shoot,front);
    }
    public Path FRONT_TO_COLLECT2(){
        return line(front,collect2).linear(front,collect2);
    }
    public Path COLLECT2_TO_TURN2(){
        return line(collect2,turn2).linear(collect2,turn2);
    }
    public  Path TURN2_TO_DIAGONAL(){
        return line(turn2,diagonal).linear(turn2,diagonal);
    }
    public  Path DIAGONAL_TO_COLLECT3(){
        return line(diagonal,collect3).linear(diagonal,collect3);
    }
    public Path DIAGONAL_TO_SHOOT2(){
        return line(collect3,shoot2).linear(collect3,shoot2);
    }


    public void init(){
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(StartPose);

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

    }

    public void start(){
        shooter.powerUp();
        Scheduler.schedule(rotine);
    }


    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

    }

}
