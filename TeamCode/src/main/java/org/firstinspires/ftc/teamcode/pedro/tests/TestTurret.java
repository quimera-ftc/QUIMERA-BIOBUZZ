package org.firstinspires.ftc.teamcode.pedro.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Agropecuaria;
import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Tower;

@TeleOp(name="CALIBRATE TURRET", group="Tests")
public class TestTurret extends OpMode {
    Intake intake;
    Agropecuaria agro;
    DriveTrain driveTrain;
    Tower tower;
    boolean flower = false;
    Pose startPose = new Pose(132.6505617977528, 133.92247191011242, Math.toRadians(270));
    Follower follower;

    @Override
    public void init() {

        follower = Constants.create(hardwareMap);
        tower = new Tower(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
        agro = new Agropecuaria(hardwareMap);
        agro.up();
        follower.setPose(startPose);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        follower.update();

        Pose currentPose = follower.pose();
        tower.update(currentPose);
    }
}