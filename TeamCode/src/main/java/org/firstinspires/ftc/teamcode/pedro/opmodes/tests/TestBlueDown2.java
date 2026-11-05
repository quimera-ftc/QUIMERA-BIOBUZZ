package org.firstinspires.ftc.teamcode.pedro.opmodes.tests;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Turret;

@Autonomous
    public class TestBlueDown2 extends LinearOpMode {

        private Follower follower;
        Turret turret;

        private final PoseFactory poseFactory = PoseFactory.degrees();

        private final Pose biobuzzmatesc = poseFactory.of(78.5051, 8.889, 90);
        private final Pose path1 = poseFactory.of(91.0176, 37.9776, 270);
        private final Pose point2Start = poseFactory.of(91.0176, 37.9776, 270);
        private final Pose point2 = poseFactory.of(93.8607, 16.6075, 270);
        private final Pose point3Start = poseFactory.of(93.8607, 16.6075, 270);
        private final Pose point3 = poseFactory.of(78.6305, 9.7349, 90);
        private final Pose point4Start = poseFactory.of(78.6305, 9.7349, 90);
        private final Pose point4 = poseFactory.of(129.4537, 17.7859, 90);

        // Autonomous routine
        public Command autoRoutine() {
            return parallel(
                    sequential(
                    follow(follower, path1()),
                    follow(follower, path2()),
                    follow(follower, path3()),
                    follow(follower, path4())
            ),
                    turret.autoaim(follower::pose));
        }

        @Override
        public void runOpMode() {
            Scheduler.reset();
            follower = Constants.create(hardwareMap);
            follower.setPose(biobuzzmatesc);
            follower.update();
            turret = new Turret(hardwareMap, biobuzzmatesc, Constants.Alliance.BLUE);



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
            return Paths.line(biobuzzmatesc, path1).linear(biobuzzmatesc, path1);
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
}

