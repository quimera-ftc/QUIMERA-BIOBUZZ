package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class shooterEvertonJoao {

    intakeEvertonJoao intake;
    DcMotor shooterRight, shooterLeft;
    public double power = 1, minPower = 0;
    public boolean shooting = false;
    public ElapsedTime timer = new ElapsedTime();



    public shooterEvertonJoao(HardwareMap hardwareMap) {

        shooterRight = hardwareMap.get(DcMotor.class, "sr");
        shooterLeft = hardwareMap.get(DcMotor.class, "sl");

        intake = new intakeEvertonJoao(hardwareMap);

    }

    public void turnShooterOn() {
        shooterRight.setPower(power);
        shooterLeft.setPower(power);

        shooting = true;
    }

    public void turnShooterOff() {
        shooterRight.setPower(minPower);
        shooterLeft.setPower(minPower);

        shooting = false;
    }

    public void shooter() {
        turnShooterOn();

    }
}
