package org.firstinspires.ftc.teamcode.pedro.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Agropecuaria;
import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Turret;

@TeleOp(name="CALIBRATE TURRET", group="Tests")
public class TestTurret extends OpMode {
    Intake intake;
    Agropecuaria agro;
    DriveTrain driveTrain;
    Turret turret;
    boolean flower = false;
    Pose startPose = new Pose(132.6505617977528, 133.92247191011242, Math.toRadians(270));
    Follower follower;

    @Override
    public void init() {

        follower = Constants.create(hardwareMap);
        turret = new Turret(hardwareMap, startPose, Constants.Alliance.RED);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
        agro = new Agropecuaria(hardwareMap);
        agro.up();
        follower.setPose(startPose);

        turret.setMode(Turret.Mode.IDLE);
    }

    @Override
    public void start() {
        turret.setMode(Turret.Mode.MANUAL);
    }

    @Override
    public void loop() {
        follower.update();

        Pose currentPose = follower.pose();
        turret.update(currentPose);


        if (gamepad1.right_bumper) {
            turret.setManualPower(0.5);
        } else if (gamepad1.left_bumper) {
            turret.setManualPower(-0.5);
        } else turret.setManualPower(0);

        if (gamepad1.dpadRightWasPressed()) {

            if (turret.getMode() == Turret.Mode.MANUAL) {
                turret.setMode(Turret.Mode.TRACKING);
            } else turret.setMode(Turret.Mode.MANUAL);

        }

        telemetry.addData("target angle", turret.computeTargetAngle());
        telemetry.addData("mode", turret.getMode());
        telemetry.addData("Turret X", turret.getTurretPose().x());
        telemetry.addData("Turret Y", turret.getTurretPose().y());

        telemetry.addData("1. Alvo Local (Target)", turret.computeTargetAngle());
        telemetry.addData("2. Posicao Local (Raw)", turret.getRawAngle());
//
        telemetry.addData("3. Erro Atual", turret.normalizeDegrees(turret.computeTargetAngle() - turret.getRawAngle()));
        telemetry.addData("4. PID Power", turret.calculatePower(0.02));

        driveTrain.driveStick();
        telemetry.update();
    }
}