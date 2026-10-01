package org.firstinspires.ftc.teamcode.pedro.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class testIntakesLateral extends OpMode {

    DcMotor intakeF, intakeL;
    boolean intakeOn = false;

    @Override
    public void loop() {

        if (gamepad1.aWasPressed()) {

            if (!intakeOn) {
                intakeF.setPower(1);
                intakeL.setPower(1);
                intakeOn = true;
            } else {
                intakeL.setPower(0);
                intakeF.setPower(0);
                intakeOn = false;
            }

        }

    }

    @Override
    public void init() {
        intakeF = hardwareMap.get(DcMotor.class, "IF");
        intakeL = hardwareMap.get(DcMotor.class, "IL");

        intakeF.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void start() {
        intakeOn = false;

    }
}
