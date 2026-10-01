package org.firstinspires.ftc.teamcode.pedro.tests;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class testFlyWheel extends OpMode{

    DcMotor shooterRight, shooterLeft;

    double maxPower = 1.0, minPower = 0.0;



    @Override
    public void start() {


        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");

        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void init() {

    }
    @Override
    public void loop() {

        if (gamepad1.right_trigger_pressed) {

            shooterRight.setDirection(DcMotor.Direction.REVERSE);
            shooterLeft.setDirection(DcMotor.Direction.FORWARD);
            shooterLeft.setPower(maxPower);
            shooterRight.setPower(maxPower);
        } else {
            shooterRight.setPower(minPower);
            shooterLeft.setPower(minPower);
        }

        if (gamepad1.left_trigger_pressed) {

            shooterRight.setDirection(DcMotor.Direction.FORWARD);
            shooterLeft.setDirection(DcMotor.Direction.FORWARD);
            shooterRight.setPower(maxPower);
            shooterLeft.setPower(maxPower);
        } else {
            shooterRight.setPower(minPower);
            shooterLeft.setPower(minPower);
        }


    }


}