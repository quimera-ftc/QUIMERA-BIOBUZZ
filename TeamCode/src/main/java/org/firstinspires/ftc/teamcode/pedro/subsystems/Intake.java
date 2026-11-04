package org.firstinspires.ftc.teamcode.pedro.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotor intake;
    public Agropecuaria agro;
    double maxPower = 0.95;
    private boolean isOn = false;
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        agro = new Agropecuaria(hardwareMap);

    }

    public void in() {
        isOn = true;
        intake.setPower(maxPower);

    }

    public void out() {
        isOn = true;
        intake.setPower(-maxPower);
    }

    public void stop() {
        isOn = false;
        intake.setPower(0);

    }

    public void toggle() {
        if (!isOn) {
            in();
        } else {
            stop();
        }
    }

    public void toggleReversed() {
        if (!isOn) {
            out();
        } else {
            stop();
        }

    }

    public boolean isOn(){
        return isOn;
    }

    //Metodos em Commands-----------------------------------------

    //metodo p intake no autoooo
    public Command intakePP() {
        return parallel(
                instant(() -> in())//,
                //instant(() -> agro.down())
        );

    }

    public Command intakestopPP() {
        return parallel(
                instant(() -> stop())//,
                //instant(() -> agro.up())
        );
    }

    public Command togglePP() {
        return sequential(
                instant(() -> toggle())
        );
    }

}