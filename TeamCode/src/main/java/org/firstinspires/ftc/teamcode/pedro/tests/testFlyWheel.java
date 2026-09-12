package org.firstinspires.ftc.teamcode.pedro.tests;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

public class testFlyWheel extends OpMode {

    DcMotor fr, fl;

    double maxPower = 1.0;

    public void loop() {

        fr = hardwareMap.get(DcMotorEx.class, "fr");
        fl = hardwareMap.get(DcMotorEx.class, "fl");

        if(gamepad1.aWasPressed()) {
            fr.setPower(maxPower);
            fl.setPower(maxPower);
        }


    }

    public void start() {


    }

    @Override
    public void init() {

    }
}
