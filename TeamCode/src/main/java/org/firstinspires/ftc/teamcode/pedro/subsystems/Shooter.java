package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {

    DcMotorEx rs, ls;
    public double maxpower = 0.80, minpower = 0;
    PIDFCoefficients controller=  new PIDFCoefficients(0.5,0,0,0.5);
    //-- GnHk28 has left the Commits

    public Shooter(HardwareMap hardwareMap) {
        ls = hardwareMap.get(DcMotorEx.class, "ls");
        rs = hardwareMap.get(DcMotorEx.class, "rs");

        ls.setDirection(DcMotorSimple.Direction.REVERSE);
        rs.setDirection(DcMotorSimple.Direction.REVERSE);

        ls.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, controller);
        rs.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, controller);


        ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    //
    public void shoot() {
        rs.setPower(maxpower);
        ls.setPower(maxpower);
    }

    public void stop() {
        rs.setPower(minpower);
        ls.setPower(minpower);
    }
}
