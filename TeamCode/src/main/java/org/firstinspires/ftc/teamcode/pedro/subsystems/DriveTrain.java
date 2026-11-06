package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {

   public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    Gamepad gamepad1;
    public DriveTrain(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.gamepad1 = gamepad1;

        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);


        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    double spin, strafe, drive;
    public void driveStick() {

        drive = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x * 1.1f; // inputs invertidos
        spin = gamepad1.right_stick_x;

        if (Math.abs(drive) < 0.1) {
            drive = 0.0;
        }
        if (Math.abs(strafe) < 0.1) {
            strafe = 0.0;
        }
        if (Math.abs(spin) < 0.1) {
            spin = 0.0;
        }
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(spin), 1);

        double frontLeftPower  = (drive + strafe + spin) / denominator;
        double backLeftPower   = (drive - strafe + spin) / denominator;
        double frontRightPower = (drive - strafe - spin) / denominator;
        double backRightPower  = (drive + strafe - spin) / denominator;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

}


