package org.firstinspires.ftc.teamcode.pedro.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.subsystems.Servos;

@TeleOp
public class servoTesting extends OpMode {

    Servos locker;

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            locker.locking();
        }

        if (gamepad1.bWasPressed()) {
            locker.notLocking();
        }
    }

    @Override
    public void init() {

        locker = new Servos(hardwareMap);


    }
}
