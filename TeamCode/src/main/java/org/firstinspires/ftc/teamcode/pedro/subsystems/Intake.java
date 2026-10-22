package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotor intake;
    private boolean isOn = false;
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    public void in() {
        isOn = true;
        intake.setPower(1);
    }

    public void out() {
        isOn = true;
        intake.setPower(-1);
    }

    public void stop() {
        isOn = false;
        intake.setPower(0);
    }

    public void toggle() {
        if (!isOn) {
            in();
        } else stop();
    }

    public boolean isOn(){
        return isOn;
    }
}