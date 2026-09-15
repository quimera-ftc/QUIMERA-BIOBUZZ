package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter2 {

    public DcMotor sr, sl;
    public double shooterPower = 0.90;
    public boolean shooterOn = false;


    public Shooter2(HardwareMap hardwareMap) {
        sr = hardwareMap.get(DcMotor.class, "sr");
        sl = hardwareMap.get(DcMotor.class, "sl");

        sr.setDirection(DcMotor.Direction.FORWARD);
        sl.setDirection(DcMotor.Direction.FORWARD);

    }

    public void shooterOn() {
        sr.setPower(shooterPower);
        sl.setPower(shooterPower);
        shooterOn  = true;
    }

    public void shooterOff() {
        sr.setPower(0);
        sl.setPower(0);
        shooterOn  = false;
    }


}
