package org.firstinspires.ftc.teamcode.pedro.tests;

import static org.firstinspires.ftc.teamcode.pedro.Constants.BangBangCoeffiecients.kBB;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Agropecuaria;
import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Turret;

@TeleOp(name="TEST MECHANISMS", group="Tests")
public class Test_MECHANISMS extends OpMode {
    Intake intake;
    DriveTrain driveTrain;
    Turret turret;
    Shooter shooter;
    Agropecuaria agro;
    Follower follower;
    Pose startPose = new Pose(132.6505617977528, 133.92247191011242, Math.toRadians(270));
    boolean flower = true;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        intake = new Intake(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
        shooter = new Shooter(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, startPose, Constants.Alliance.BLUE);
        agro = new Agropecuaria(hardwareMap);
        agro.up();
        follower.setPose(new Pose(132.6505617977528, 133.92247191011242, Math.toRadians(270)));
    }

    @Override
    public void loop() {
        follower.update();

        Pose currentPose = follower.pose();
        turret.update(currentPose);

        if (gamepad1.aWasPressed()) {
            intake.toggle();
        }

        if (gamepad1.yWasPressed()) {
            intake.toggleReversed();
        }

        if (gamepad1.right_trigger_pressed) {
            shooter.shooterPower(kBB);
            shooter.updateBangBang();

        } else {
            shooter.stop();
        }

        if (gamepad1.xWasPressed()) {

            if (!flower) {
                agro.up();
            } else agro.down();

            flower = !flower;

        }

        if (gamepad1.right_bumper) {
            turret.setManualPower(0.75);
        } else if (gamepad1.left_bumper) {
            turret.setManualPower(-0.75);
        } else turret.setManualPower(0);

        if (gamepad1.dpadRightWasPressed()) {

            if (turret.getMode() == Turret.Mode.MANUAL) {
                turret.setMode(Turret.Mode.TRACKING);
            } else turret.setMode(Turret.Mode.MANUAL);

        }

        driveTrain.driveStick();

        telemetry.addData("intake", intake.isOn() ? "LIGADO" : "DESLIGADO");
        telemetry.addData("shooter", shooter.isOn() ? "LIGADO" : "DESLIGADO");
        telemetry.addData("turret", turret.getMode());
        telemetry.addLine("--ODOMETRIA--");
        telemetry.addData("x", currentPose.x());
        telemetry.addData("y", currentPose.y());
        telemetry.addData("heading", Math.toDegrees(currentPose.heading()));

        telemetry.update();
    }
}
