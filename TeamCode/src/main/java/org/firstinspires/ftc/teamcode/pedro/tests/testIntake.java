package org.firstinspires.ftc.teamcode.pedro.tests;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class testIntake {


    DcMotor intakeLow, intakeHigh;
    double power = 1.0, direction = 1.0, minPower = 0;
    boolean powered = false;

    public testIntake(HardwareMap hardwareMap) {

        intakeLow = hardwareMap.get(DcMotor.class, "intakeLow");
        intakeHigh = hardwareMap.get(DcMotor.class, "intakeHigh");

        intakeLow.setDirection(DcMotor.Direction.FORWARD);
        intakeHigh.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void collectOn() {

        intakeLow.setPower(power * direction);
        powered = true;

    }

    public void reverse() {

        direction = -direction;

        if (powered) {
            intakeLow.setPower(direction * power);
        }
    }

    public void collectOff(){

        intakeLow.setPower(minPower);
        powered = false;
    }

    public void toggle() {
        if (!powered) {
            collectOn();
        } else {
            collectOff();
        }
    }

    public void setPower(double power) {
        this.power = power;
        if (powered) {
            intakeLow.setPower(direction * power);
        }
    }
}
