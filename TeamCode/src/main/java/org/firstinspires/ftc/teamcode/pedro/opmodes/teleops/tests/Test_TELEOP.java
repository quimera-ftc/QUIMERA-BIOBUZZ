package org.firstinspires.ftc.teamcode.pedro.opmodes.teleops.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Agropecuaria;
import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Tower;

@TeleOp(name="TELEOP", group="Tests")
public class Test_TELEOP extends OpMode {
    Intake intake;
    DriveTrain driveTrain;
    Tower tower;
    Shooter shooter;
    Agropecuaria agro;
    Follower follower;
    Pose startPose = new Pose(132.6505617977528, 133.92247191011242, Math.toRadians(270));


    //public boolean isOn = false;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        intake = new Intake(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
        shooter = new Shooter(hardwareMap);
        tower = new Tower(hardwareMap);
        agro = new Agropecuaria(hardwareMap);

        follower.setPose(new Pose(132.6505617977528, 133.92247191011242, Math.toRadians(270)));
    }

    @Override
    public void loop() {
        follower.update();

        Pose currentPose = follower.pose();

        if (gamepad1.aWasPressed()) {
            intake.toggle();
        }

        if (gamepad1.yWasPressed()) {
            intake.toggleReversed();
        }

        if (gamepad1.right_trigger_pressed) {
            shooter.powerUp();
        } else {
            shooter.stop();
        }

        if (gamepad1.left_trigger_pressed) {
            shooter.shooterPower(2100);
        } else {
            shooter.stop();
        }

        if (gamepad1.xWasPressed()) {
            agro.down();
        }else{
            agro.up();
        }

//        if (isOn) {
//            intake.in();
//        } else {
//            intake.stop();
//        } //porquice

        driveTrain.driveStick();


        telemetry.addData("intake", intake.isOn() ? "LIGADO" : "DESLIGADO");
        telemetry.addData("shooter", shooter.isOn() ? "LIGADO" : "DESLIGADO");
        telemetry.addLine("--ODOMETRIA--");
        telemetry.addData("x", currentPose.x());
        telemetry.addData("y", currentPose.y());
        telemetry.addData("heading", Math.toDegrees(currentPose.heading()));
        telemetry.addLine("--TORRETA--");

        telemetry.update();
    }
}
