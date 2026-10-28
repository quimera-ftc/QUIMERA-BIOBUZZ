package org.firstinspires.ftc.teamcode.pedro.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotor intake;
    public ServoTaura servo;
    private boolean isOn = false;
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");

        servo = new ServoTaura(hardwareMap);
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
        servo.up();
    }

    public void toggle() {
        if (!isOn) {
            in();
        } else stop();
    }

    public boolean isOn(){
        return isOn;
    }

    //metodo p intake no autoooo
    public Command intakePP() {
        return sequential(
                instant(() -> in()),
                instant(() -> servo.down())
        );

    }
}