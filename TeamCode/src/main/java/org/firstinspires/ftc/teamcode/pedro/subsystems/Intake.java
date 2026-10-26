package org.firstinspires.ftc.teamcode.pedro.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private final DcMotor intake;
    Servo agroLeft, agroRight;
    public double rightAGRODOWNING = 0.80, rightAGROUPPING = 0.25;
    public double leftAGRODOWNING = 1.0, leftAGROUPPING = 0.15;
    public double power = 0.95;
    public double minPower = 0;
    private boolean isOn = false, isAgro = false;
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        agroLeft = hardwareMap.get(Servo.class, "al");
        agroRight = hardwareMap.get(Servo.class, "ar");

        agroLeft.setDirection(Servo.Direction.FORWARD);
        agroRight.setDirection(Servo.Direction.REVERSE);
    }

    public void in() {
        isOn = true;
        intake.setPower(power);
    }

    public void out() {
        isOn = true;
        intake.setPower(-power);
    }

    public void stop() {
        isOn = false;
        intake.setPower(minPower);
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
                instant(() -> in())

        );

    }

    //-----------------------servo--------------------------

    public void agroDown() {
        agroLeft.setPosition(leftAGRODOWNING);
        agroRight.setPosition(rightAGRODOWNING);

        isAgro = true;

    }

    public void agroUp() {
        agroLeft.setPosition(leftAGROUPPING);
        agroRight.setPosition(rightAGROUPPING);

        isAgro = false;

    }

    public void toggleAgro() {
        if (!isAgro) {
            agroUp();
        } else {
            agroDown();
        }

        isAgro = !isAgro;
    }
}