package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {

    DcMotor frontLeft, frontRight, backLeft, backRight;
    Gamepad gamepad1;
    public DriveTrain(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.gamepad1 = gamepad1;

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

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
        spin = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;
        drive = gamepad1.left_stick_y;

        if (Math.abs(drive) < 0.1) {
            drive = 0.0;
        }
        if (Math.abs(strafe) < 0.1) {
            strafe = 0.0;
        }
        if (Math.abs(spin) < 0.1) {
            spin = 0.0;
        }

        double frontLeftPower = drive + strafe + spin;
        double frontRightPower = drive - strafe - spin;
        double backLeftPower = drive - strafe + spin;
        double backRightPower = drive + strafe - spin;

        frontLeft.setPower(frontRightPower);
        frontRight.setPower(frontLeftPower);
        backLeft.setPower(backRightPower);
        backRight.setPower(backLeftPower);
    }

}
