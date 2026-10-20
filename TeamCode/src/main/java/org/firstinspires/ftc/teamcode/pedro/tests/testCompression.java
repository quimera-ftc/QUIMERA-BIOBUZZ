package org.firstinspires.ftc.teamcode.pedro.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class testCompression extends OpMode {

    DcMotor shooterRight, shooterLeft;

    public double maxPower = 1.0, reversed = -1.0, minPower = 0.0;

    @Override
    public void init() {

        shooterRight = hardwareMap.get(DcMotor.class, "sr");
        shooterLeft = hardwareMap.get(DcMotor.class, "sl");

        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    @Override
    public void loop() {

        if (gamepad1.right_trigger_pressed) {
            shooterRight.setPower(maxPower);
            shooterLeft.setPower(maxPower);
        } else {
            shooterRight.setPower(minPower);
            shooterLeft.setPower(minPower);
        }

        if (gamepad1.left_trigger_pressed) {
            shooterRight.setPower(reversed * maxPower);
            shooterLeft.setPower(reversed * maxPower);
        } else {
            shooterRight.setPower(minPower);
            shooterLeft.setPower(minPower);

        }

    }

    @Override
    public void start() {

    }

}
