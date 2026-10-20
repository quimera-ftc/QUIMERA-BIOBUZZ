package org.firstinspires.ftc.teamcode.pedro.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class testIntake extends OpMode {

    boolean on = false;

    DcMotor intakeLow, intakeHigh;

    @Override
    public void init() {
        intakeLow = hardwareMap.get(DcMotor.class, "intakeLow");


        intakeLow.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void loop() {

        if (gamepad1.aWasPressed()) {

            if (!on) {
                intakeLow.setPower(1);
            } else {
                intakeLow.setPower(0);
            }

            on = !on;
        }

    }
}



