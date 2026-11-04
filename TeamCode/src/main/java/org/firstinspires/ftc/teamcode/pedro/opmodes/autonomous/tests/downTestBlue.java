package org.firstinspires.ftc.teamcode.pedro.opmodes.autonomous.tests;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class downTestBlue extends LinearOpMode {

    private Follower follower;
    Shooter shooter;
    Intake intake;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose biobuzzmatesc = poseFactory.of(82.6039, 8.1762, 90);
    private final Pose path1 = poseFactory.of(94.2254, 21.7589, -90);
    private final Pose path1Control1 = poseFactory.of(89.5237, 52.8836, 0);
    private final Pose point2Start = poseFactory.of(94.2254, 21.7589, -90);
    private final Pose point2 = poseFactory.of(94.4418, 13.4704, -90);
    private final Pose point3Start = poseFactory.of(94.4418, 13.4704, -90);
    private final Pose point3 = poseFactory.of(83.0864, 8.7627, 180);
    private final Pose point4Start = poseFactory.of(83.0864, 8.7627, 90);
    private final Pose point4 = poseFactory.of(82.5094, 128.8409, 0);
    private final Pose point5Start = poseFactory.of(82.5094, 128.8409, 0);
    private final Pose point5 = poseFactory.of(134.2853, 135.7414, 0);
    private final Pose point6Start = poseFactory.of(134.2853, 135.7414, 0);
    private final Pose point6 = poseFactory.of(96.6269, 121.4958, -60);
    private final Pose point7Start = poseFactory.of(96.6269, 121.4958, -60);
    private final Pose point7 = poseFactory.of(111.4148, 93.481, 0);
    private final Pose point8Start = poseFactory.of(111.4148, 93.481, 0);
    private final Pose point8 = poseFactory.of(129.7837, 93.5798, 0);
    private final Pose point9Start = poseFactory.of(129.7837, 93.5798, 0);
    private final Pose point9 = poseFactory.of(132.1269, 51.2897, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7()),
                follow(follower, path8()),
                follow(follower, path9())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(biobuzzmatesc);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.curve(biobuzzmatesc, path1Control1, path1).linear(biobuzzmatesc, path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.line(point7Start, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.line(point8Start, point8).linear(point8Start, point8);
    }

    public Path path9() {
        return Paths.line(point9Start, point9).linear(point9Start, point9);
    }
}