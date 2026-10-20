package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intakeEvertonJoao {

    DcMotor intakeLeft, intakeRight;
    public boolean collecting = false;
    public double power = 1, minPower = 0, direction = 1;

    public intakeEvertonJoao(HardwareMap hardwareMap) {

        intakeLeft = hardwareMap.get(DcMotor.class, "il");
        intakeRight = hardwareMap.get(DcMotor.class, "ir");

        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        intakeRight.setDirection(DcMotor.Direction.FORWARD);
    }

    public void turnIntakeOn() {

        intakeLeft.setPower(power * direction);
        intakeRight.setPower(power * direction);

        collecting = true;
    }

    public void turnIntakeOff() {

        intakeLeft.setPower(minPower * direction);
        intakeRight.setPower(minPower * direction);

        collecting = false;
    }

    public void reverse() {
        direction = -direction;

        if (collecting) {
            intakeLeft.setPower(power * direction);
            intakeRight.setPower(power * direction);
        }
    }

    public void toggle() {
        if (!collecting) {
            turnIntakeOn();
        } else {
            turnIntakeOff();
        }
    }

}
