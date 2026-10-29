package org.firstinspires.ftc.teamcode.pedro.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Agropecuaria {

    ServoTaura left, right;
    public Agropecuaria(@NonNull HardwareMap hardwareMap) {

        Servo s1, s2;
        s1 = hardwareMap.get(Servo.class, "flow");
        s2 = hardwareMap.get(Servo.class, "flow1");

        left = new ServoTaura(s1);
        right = new ServoTaura(s2);

        s1.setDirection(Servo.Direction.FORWARD );
        s2.setDirection(Servo.Direction.REVERSE);


    }
    public void up() {
        left.setPosition(0);

    }
    public void down() {
        right.setPosition(0.5);
    }
}
