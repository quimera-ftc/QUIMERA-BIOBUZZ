package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotor intake;
    private boolean isOn = false;
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    public void turnOn() {
        isOn = true;
        intake.setPower(1);
    }

    public void turnOff() {
        isOn = false;
        intake.setPower(0);
    }

    public void toggle() {
        if (!isOn) {
            turnOn();
        } else turnOff();
    }

    public boolean isOn(){
        return isOn;
    }
}