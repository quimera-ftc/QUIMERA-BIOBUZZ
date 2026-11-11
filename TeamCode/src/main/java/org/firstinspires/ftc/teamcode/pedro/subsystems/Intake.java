package org.firstinspires.ftc.teamcode.pedro.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;

import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.IntakeConstants.activeAgroPos;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.IntakeConstants.idleAgroPos;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.math.RampaPotencia;
import org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.IntakeConstants;

public class Intake {

    public final DcMotorEx intake;
    public final ServoTaura agro;
    double alvo = 0;
    private final RampaPotencia rampa = new RampaPotencia(3.3, 10.0);
    private boolean isOn = false;
    public boolean flower = false;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        Servo servoAgro = hardwareMap.get(Servo.class, "agro");
        agro = new ServoTaura(servoAgro);

        intake.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void in(double power) {
        isOn = true;
        alvo = power;
    }

    public void out(double power) {
        isOn = true;
        alvo = power;
    }

    public void stop() {
        isOn = false;
        alvo = 0;
    }

    public void update() {
        intake.setPower(rampa.calcular(alvo));
    }

    public void toggle(double wantedCoefficient) {
        if (isOn) {
            in(wantedCoefficient);
        } else {
            stop();
        }

        isOn = !isOn;
    }

    public void collectFlower(boolean collect) {
        if (collect) {
            agro.setPosition(activeAgroPos);
            flower = true;
        }
        else {
            agro.setPosition(idleAgroPos);
            flower = false;
        }

    }

    public void toggleReversed(double wantedCoefficient) {
        if (isOn) {
            out(-wantedCoefficient);
        } else {
            stop();
        }

        isOn = !isOn;
    }

    public boolean isOn(){
        return isOn;
    }

    //Metodos em Commands-----------------------------------------

    //metodo p intake no autoooo
    public Command intakePP() {
        return parallel(
                instant(() -> in(IntakeConstants.intakeCollecting)),
                instant(() -> collectFlower(true))
        );

    }

    public Command intakeOnlyUse() {
        return parallel(
                instant(() -> in(IntakeConstants.intakeCollecting))
        );

    }

    public Command intakestopPP() {
        return parallel(
                instant(this::stop),
                instant(() -> collectFlower(false))
        );
    }


}