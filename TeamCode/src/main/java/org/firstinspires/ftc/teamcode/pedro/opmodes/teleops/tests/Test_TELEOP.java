package org.firstinspires.ftc.teamcode.pedro.opmodes.teleops.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;

public class Test_TELEOP extends OpMode {

    Intake intake;
    DriveTrain driveTrain;
    Shooter shooter;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
        shooter = new Shooter(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.aWasPressed()) {
            intake.toggle();
        }

        if (gamepad1.right_trigger_pressed) {
            shooter.shoot();
        } else {
            shooter.stop();
        }


        driveTrain.driveStick();

    }
}
