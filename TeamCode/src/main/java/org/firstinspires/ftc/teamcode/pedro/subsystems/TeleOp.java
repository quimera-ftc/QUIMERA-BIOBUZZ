package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {

    Intake2 intake;
    DriveTrain driveTrain;
    Shooter2 shooter;
    Timer runtime;
    boolean shooting = false;
    double shootingTime = 0.0;

    @Override
    public void init() {
        runtime = new Timer();
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
        intake = new Intake2(hardwareMap);
        shooter = new Shooter2(hardwareMap);

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        if (gamepad1.aWasPressed()) {
            intake.toggleBottom();
        }

        if (gamepad1.xWasPressed()) {
            intake.toggleTop();
        }

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
            shooting = false;
            intake.stopTop();
        }


//----------------------------------------------------------

        driveTrain.driveStick();
    }

}
