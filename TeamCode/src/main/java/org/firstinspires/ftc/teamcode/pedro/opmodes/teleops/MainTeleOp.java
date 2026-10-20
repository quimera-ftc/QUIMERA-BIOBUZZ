package org.firstinspires.ftc.teamcode.pedro.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;

@TeleOp
public class MainTeleOp extends OpMode {

    DcMotor intakeTop, intakeLow;
    public double power = 1, minPower = 0, direction = 1;
    public boolean powered = false;

    @Override
    public void init() {
        intakeTop = hardwareMap.get(DcMotor.class, "intakeTop");
        intakeLow = hardwareMap.get(DcMotor.class, "intakeLow");

    }

    @Override
    public void loop() {

        if (gamepad1.left_trigger_pressed) {
            intakeLow.setPower(power * direction);
            intakeTop.setPower(power * direction);
            powered = true;
        } else {
            intakeTop.setPower(minPower);
            intakeLow.setPower(minPower);
        }

        if (gamepad1.right_trigger_pressed) {
            direction = -direction;
            if (powered) {
                intakeTop.setPower(power * direction);
                intakeLow.setPower(power * direction);
            }
        } else {
            intakeTop.setPower(minPower);
            intakeLow.setPower(minPower);
        }
    }

    @Override
    public void start() {

    }
}
