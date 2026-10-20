package org.firstinspires.ftc.teamcode.pedro.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class testTABLEIntakes extends OpMode {

    DcMotor intake;
    boolean intakeOn = false;
    double power = 1;


    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        if (gamepad1.yWasPressed()) {
            power = -power;
        }

        if (gamepad1.aWasPressed()) {

            if (intakeOn) {
                intake.setPower(0);
                intakeOn = false;
            } else {
                intake.setPower(power);
                intakeOn = true;
            }
        }


    }


}
