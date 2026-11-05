package org.firstinspires.ftc.teamcode.pedro.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;

@TeleOp(name="TESTS VOLTAGE", group="Tests")
public class TestVoltage extends OpMode {

    DriveTrain driveTrain;
    Intake intake;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap, gamepad1);
    }

    @Override
    public void loop() {

        if (gamepad1.aWasPressed()) {
            intake.toggle();
        }


        driveTrain.driveStick();
        telemetry.update();
    }
}