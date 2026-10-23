package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    DcMotor rs, ls;
    public double maxpower = 1, minpower = 0;
    //COMENTARIOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
    public Shooter(HardwareMap hardwareMap) {
        ls = hardwareMap.get(DcMotor.class, "ls");
        rs = hardwareMap.get(DcMotor.class, "rs");

        ls.setDirection(DcMotorSimple.Direction.FORWARD);
        rs.setDirection(DcMotorSimple.Direction.REVERSE);

        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
