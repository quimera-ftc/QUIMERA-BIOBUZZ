package org.firstinspires.ftc.teamcode.pedro.opmodes.autonomous.tests;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
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
