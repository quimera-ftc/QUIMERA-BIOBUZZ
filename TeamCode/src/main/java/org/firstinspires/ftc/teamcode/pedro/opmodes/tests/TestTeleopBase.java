package org.firstinspires.ftc.teamcode.pedro.opmodes.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;

@TeleOp(name = "BASE TELEOP")

public class TestTeleopBase extends OpMode {
    DriveTrain driveTrain;
    Shooter shooter;
    Intake intake;
    Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        follower.setPose(PoseStorage.currentPose);
    }

    @Override
    public void loop() {
        follower.update();


        if (gamepad1.aWasPressed()) {
            intake.toggle();
        }

        if (gamepad1.right_trigger > 0.5) {
            shooter.setAlvo(Constants.ShooterCoefficients.ALVO_TPS);
        } else {
            shooter.setAlvo(0);
        }

        Pose currentPose = follower.pose();

        telemetry.addData("x", currentPose.x());
        telemetry.addData("y", currentPose.y());
        telemetry.addData("heading", currentPose.heading());
        telemetry.addData("on", intake.isOn());
        telemetry.addData("power intake", intake.intake.getPower());
        telemetry.addData("current intake", intake.intake.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("current fl", driveTrain.frontLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("current fr", driveTrain.frontRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("current bl", driveTrain.backLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("current br", driveTrain.backRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("current rs", shooter.rs.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("current ls", shooter.ls.getCurrent(CurrentUnit.AMPS));



        driveTrain.driveStick();
        shooter.update();
        telemetry.update();
    }
}
