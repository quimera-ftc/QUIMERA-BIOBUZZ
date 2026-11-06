package org.firstinspires.ftc.teamcode.pedro.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Intake {

    public final DcMotorEx intake;
    public Agropecuaria agro;
//    double maxPower = 0.75;
    private boolean isOn = false;
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        agro = new Agropecuaria(hardwareMap);

    }

    public void in(double power) {
        isOn = true;
        intake.setPower(power);

    }

    public void out(double power) {
        isOn = true;
        intake.setPower(power);
    }

    public void stop() {
        isOn = false;
        intake.setPower(0);

    }

    public void toggle(double wantedCoefficient) {
        if (isOn) {
            in(wantedCoefficient);
        } else {
            stop();
        }

        isOn = !isOn;
    }

    public void toggleReversed() {
        if (isOn) {
            stop();
        } else {
            out(Constants.IntakeCoefficients.intakeIdle);
        }

    }

    public boolean isOn(){
        return isOn;
    }

    //Metodos em Commands-----------------------------------------

    //metodo p intake no autoooo
    public Command intakePP() {
        return parallel(
                instant(() -> in(Constants.IntakeCoefficients.intakeCollecting))//,
                //instant(() -> agro.down())
        );

    }

    public Command intakestopPP() {
        return parallel(
                instant(() -> stop())//,
                //instant(() -> agro.up())
        );
    }


}