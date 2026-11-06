package org.firstinspires.ftc.teamcode.pedro.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;

@TeleOp(name="TESTS VOLTAGE", group="Tests")
public class TestVoltage extends OpMode {

    DriveTrain driveTrain;
    Intake intake;
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
            if(shooter.isShooting) {
                intake.toggle(Constants.IntakeCoefficients.intakeShooting);
            } else {
                intake.toggle(Constants.IntakeCoefficients.intakeCollecting);
            }

        }


        driveTrain.driveStick();
        telemetry.update();
    }
}