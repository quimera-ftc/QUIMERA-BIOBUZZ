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

    DcMotor shooterRight, shooterLeft; //intakeLow;

    double maxPower = 1.0, minPower = 0.0, direction = 1.0;
    boolean on = false, collecting = false;



    @Override
    public void start() {


        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        //intakeLow = hardwareMap.get(DcMotor.class, "intakeLow");


        //intakeLow.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);

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
            shooterLeft.setDirection(DcMotor.Direction.REVERSE);
            shooterRight.setPower(maxPower);
            shooterLeft.setPower(maxPower);
        } else {
            shooterRight.setPower(minPower);
            shooterLeft.setPower(minPower);
        }

//        if (gamepad1.yWasPressed()) {
//            direction = -direction;
//
//            if (collecting) {
//                intakeLow.setPower(maxPower * direction);
//            }
//        }
//
//        if (gamepad1.aWasPressed()) {
//
//            if (!on) {
//                intakeLow.setPower(1);
//                collecting = true;
//            } else {
//                intakeLow.setPower(0);
//                collecting = false;
//            }
//
//            on = !on;
        }


    }


//}