package org.firstinspires.ftc.teamcode.pedro.subsystems;

import static org.firstinspires.ftc.teamcode.pedro.Constants.movimentSpeed;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedro.math.RampaPotencia;

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
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);


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

    final RampaPotencia rampaDrive  = new RampaPotencia(5.0, 10.0);  // 0→1 em 0,2 s; para em 0,1 s
    final RampaPotencia rampaStrafe = new RampaPotencia(5.0, 10.0);
    final RampaPotencia rampaSpin   = new RampaPotencia(5.0, 10.0);

    public void driveStick() {
        drive = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x * 1.1f; // inputs invertidos
        spin = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(spin), 1);

        double frontLeftPower  = (drive + strafe + spin) / denominator;
        double backLeftPower   = (drive - strafe + spin) / denominator;
        double frontRightPower = (drive - strafe - spin) / denominator;
        double backRightPower  = (drive + strafe - spin) / denominator;

        if (Math.abs(drive)  < 0.1) drive  = 0;
        if (Math.abs(strafe) < 0.1) strafe = 0;
        if (Math.abs(spin)   < 0.1) spin   = 0;

        drive  = rampaDrive.calcular(drive);      // NOVO
        strafe = rampaStrafe.calcular(strafe);    // NOVO
        spin   = rampaSpin.calcular(spin);

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void driveStick(Telemetry telemetry) {
        drive = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x * 1.1f; // inputs invertidos
        spin = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(spin), 1);

        double frontLeftPower  = (drive + strafe + spin) / denominator;
        double backLeftPower   = (drive - strafe + spin) / denominator;
        double frontRightPower = (drive - strafe - spin) / denominator;
        double backRightPower  = (drive + strafe - spin) / denominator;

        if (Math.abs(drive)  < 0.1) drive  = 0;
        if (Math.abs(strafe) < 0.1) strafe = 0;
        if (Math.abs(spin)   < 0.1) spin   = 0;

        drive  = rampaDrive.calcular(drive);      // NOVO
        strafe = rampaStrafe.calcular(strafe);    // NOVO
        spin   = rampaSpin.calcular(spin);

        frontLeft.setPower(frontLeftPower * movimentSpeed);
        frontRight.setPower(frontRightPower * movimentSpeed);
        backLeft.setPower(backLeftPower * movimentSpeed);
        backRight.setPower(backRightPower * movimentSpeed);

        telemetry.addData("current fl", frontLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("current fr", frontRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("current bl", backLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("current br", backRight.getCurrent(CurrentUnit.AMPS));
    }
    public void sendTelemetry(Telemetry telemetry) {
        telemetry.addData("drive", drive);
        telemetry.addData("strafe", strafe);
        telemetry.addData("spin", spin);
    }

}


