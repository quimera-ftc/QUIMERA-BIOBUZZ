package org.firstinspires.ftc.teamcode.pedro.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.math.RampaPotencia;
import org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.IntakeConstants;

public class Intake {

    public final DcMotorEx intake;
    public Servos agro;
    double alvo = 0;
    private final RampaPotencia rampa = new RampaPotencia(3.3, 10.0);  // 0→1 em 0,3 s
//    double maxPower = 0.75;
    private boolean isOn = false;
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        agro = new Servos(hardwareMap);

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

    public void toggleReversed(double wantedCoefficient) {
        if (isOn) {
            stop();
        } else {
            out(-wantedCoefficient);
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
                instant(() -> in(IntakeConstants.intakeCollecting))//,
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