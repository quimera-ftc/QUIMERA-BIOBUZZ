package org.firstinspires.ftc.teamcode.pedro.tests;

import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake2;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter2;

@TeleOp
public class testTeleOp extends OpMode {

    Intake2 intake;
    DriveTrain driveTrain;
    Shooter2 shooter;
    Timer runtime;
    boolean shooting = false;

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            intake.reverse();
        }
        if (gamepad1.right_trigger_pressed) {
            shooter.shooterOn();

            if (!shooting) {
                runtime.reset();
            }

            if (runtime.milliseconds() > 2000) {
                intake.collectTop();
            }

            shooting = true;

        } else {
            shooter.shooterOff();
            intake.stopTop();
            shooting = false;
        }

        if (gamepad1.aWasPressed()) {
            intake.toggleBottom();
        }
        if (gamepad1.xWasPressed()) {
            intake.toggleTop();
        }
        driveTrain.driveStick();
    }

    @Override
    public void init() {
        runtime = new Timer();
        intake = new Intake2(hardwareMap);
        shooter = new Shooter2(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
    }

    public void start() {
        runtime.reset();
    }
}

