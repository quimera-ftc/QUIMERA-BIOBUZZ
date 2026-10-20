package org.firstinspires.ftc.teamcode.pedro.opmodes.teleops.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.intakeEvertonJoao;
import org.firstinspires.ftc.teamcode.pedro.subsystems.shooterEvertonJoao;

@TeleOp
public class roboEvertonJoao extends OpMode {

    DriveTrain driveTrain;
    intakeEvertonJoao intake;
    shooterEvertonJoao shooter;

    @Override
    public void loop() {

        if (gamepad1.aWasPressed()) {
            intake.toggle();
        }

        if (gamepad1.yWasPressed()) {
            intake.reverse();
        }

        if (gamepad1.right_trigger_pressed) {
            shooter.shooter();
        } else {
            shooter.turnShooterOff();
        }

        driveTrain.driveStick();
    }

    @Override
    public void start() {

    }

    @Override
    public void init() {

        driveTrain = new DriveTrain(hardwareMap, gamepad1);
        intake = new intakeEvertonJoao(hardwareMap);
        shooter = new shooterEvertonJoao(hardwareMap);

    }



}