package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake2 {

    public DcMotor topMotor, bottonMotor;

    public double topPower = 0.67, bottonPower = 0.40;
    public double direction = 1;
    public boolean topOn = false, bottonOn = false;

    public Intake2(HardwareMap hardwareMap) {
        topMotor = hardwareMap.get(DcMotor.class,"ih");
        bottonMotor = hardwareMap.get(DcMotor.class, "il");

        topMotor.setDirection(DcMotor.Direction.REVERSE);
        bottonMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void reverse() {
        direction = -direction;
        if (topOn) {
            collectTop();
        }
        if (bottonOn) {
            collectBotton();
        }
    }

    public void collectBotton() {
        bottonMotor.setPower(bottonPower * direction);
        bottonOn = true;
    }
    public void collectTop() {
        topMotor.setPower(topPower * direction);
        topOn = true;
    }

    public void stopTop() {
        topMotor.setPower(0);
        topOn = false;
    }
    public void stopBotton() {
        bottonMotor.setPower(0);
        bottonOn = false;
    }
    public void toggleTop() {
        if (topOn) {
            topMotor.setPower(0);
            topOn = false;
        } else {
            topMotor.setPower(topPower * direction);
            topOn = true;
        }
    }
    public void toggleBottom() {
        if (bottonOn) {
            bottonMotor.setPower(0);
            bottonOn = false;
        } else {
            bottonMotor.setPower(topPower * direction);
            bottonOn = true;
        }

    }

}
