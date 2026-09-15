package org.firstinspires.ftc.teamcode.pedro.tests;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

public class testFlyWheel{

    DcMotor fr,fl;

    double maxPower = 1.0, minPower = 0.0;



    public void start() {


        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");

        fr.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);

    }

    public void init() {

        fr.setPower(maxPower);
        fl.setPower(maxPower);
    }

    public void loop() {

        fr.setPower(minPower);
        fl.setPower(minPower);
    }


}
