package org.firstinspires.ftc.teamcode.pedro.opmodes.autonomous.tests;

import static com.pedropathing.ivy.groups.Groups.sequential;

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

@Autonomous
public class autoTestNataa extends OpMode {

    Shooter shooter;
    Intake intake;


    PoseFactory poseFactory = PoseFactory.degrees();
    Scheduler scheduler;
    private Follower follower;


    Pose startingPoint = poseFactory.of(86.06466512702079,133.1605080831409,270);
    Pose collect1 = poseFactory.of(132.97337962962962, 131.97106481481478, 90);
    Pose shoot2 = poseFactory.of(108.43981481481482, 14.182870370370395, 90);
    Pose collect2 = poseFactory.of(98.44675925925927, 10.232638888888886, 250);
    Pose shoot3 = poseFactory.of(108.70486111111111, 14.193287037037038, 70);
    Pose collect3 = poseFactory.of(132.8888888888889, 94.93055555555556, 0);
    Pose shoot4 = poseFactory.of(104.5462962962963, 125.4074074074074, 300);
    Pose park = poseFactory.of(133.47685185185188, 52.019675925925924, 270);


//    public Path START_TO_COLLECT1() {
//
//
//    }


    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startingPoint);

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

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


        );
    }


}
