package org.firstinspires.ftc.teamcode.pedro.opmodes.calibration;

import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.IntakeConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


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
        shooter = new Shooter(hardwareMap, intake);
    }

    @Override
    public void loop() {

        if (gamepad1.aWasPressed()) {
            if(shooter.isShooting) {
                intake.toggle(intakeShooting);
            } else {
                intake.toggle(intakeCollecting);
            }

        }


        driveTrain.driveStick();
        intake.update();
        telemetry.update();
    }
}