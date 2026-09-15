package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {

    DcMotor intakeHigh;//shooterRight, shooterLeft;
    public double power = -0.85, minPower = 0.0, direction = 1.0, turretPower = 1.0;
    public boolean powered = false;
    public ElapsedTime timer = new ElapsedTime();

    public Shooter(HardwareMap hardwareMap) {

        intakeHigh = hardwareMap.get(DcMotor.class, "ih");
        //shooterRight = hardwareMap.get(DcMotor.class, "sr");
        //shooterLeft = hardwareMap.get(DcMotor.class, "sl");

        intakeHigh.setDirection(DcMotor.Direction.FORWARD);
        //shooterLeft.setDirection(DcMotor.Direction.FORWARD);
        //shooterRight.setDirection(DcMotor.Direction.FORWARD);
    }

    public void conductOn() {

        intakeHigh.setPower(power);
        powered = true;

    }

    public void reverse() {
        direction = -direction;

        if (powered) {
            intakeHigh.setPower(direction * power);
        }
    }

    public void conductOff() {

        intakeHigh.setPower(minPower);
        powered = false;
    }

//    public void turnShooterOn() {
//        powered = true;
//        shooterLeft.setPower(1);
//        shooterRight.setPower(1);
//    }

//    public void shooting() {
//
//        timer.reset();
//
//        conduct();
//        if (timer.seconds() > 1.0 && powered) {
//            turnShooterOn();
//        }
//        conductOff();
//    }   //asterisco

//    public void turnShootOff() {
//        powered = false;
//        shooterLeft.setPower(minPower);
//        shooterRight.setPower(minPower);
//    }

}
