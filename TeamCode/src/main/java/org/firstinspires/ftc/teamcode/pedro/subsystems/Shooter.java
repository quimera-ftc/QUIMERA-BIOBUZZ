package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;

public class Shooter {

    Intake intake;

    DcMotorEx rs, ls;
    public double maxpower = 1, minpower = 0;
    public double start, now;
    boolean isOn = false;
    public ElapsedTime runtime = new ElapsedTime();
    PIDFCoefficients controller=  new PIDFCoefficients(0.5,0,0,0.5);
    //-- GnHk28 has left the Commits

    public Shooter(HardwareMap hardwareMap) {
        ls = hardwareMap.get(DcMotorEx.class, "ls");
        rs = hardwareMap.get(DcMotorEx.class, "rs");

        ls.setDirection(DcMotorSimple.Direction.FORWARD);
            rs.setDirection(DcMotorSimple.Direction.FORWARD);

        ls.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, controller);
        rs.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, controller);


//        ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        rs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake = new Intake(hardwareMap);
    }

    public boolean isOn() {
        return isOn;
    }

    public void powerUp() {
        rs.setPower(maxpower);
        ls.setPower(maxpower);

        isOn = true;
    }

    //metodo p shootar no autooooo
    public Command shootPP() {
        return sequential(
                instant(() -> powerUp()),
                instant(() -> intake.in()),
                waitMs(2000),
                instant(() -> {
                    intake.stop();
                })
        );
    }

    //metodo p shootar no telop
    public void shoot() {
        start = runtime.milliseconds();

        while (runtime.seconds() - start < 3000) {
            now = runtime.milliseconds() - start;

            if (now <= 2000) {
                powerUp();
                continue;
            }
            intake.in();
        }
        intake.stop();
        stop();
    }

    public void stop() {
        rs.setPower(0);
        ls.setPower(0);
        isOn = false;
    }
}
