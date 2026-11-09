package org.firstinspires.ftc.teamcode.pedro.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {
    ServoTaura trava;

    public Servos(@NonNull HardwareMap hardwareMap) {

        Servo s1;

        s1 = hardwareMap.get(Servo.class, "trava");


        trava = new ServoTaura(s1);

        s1.setDirection(Servo.Direction.FORWARD);


    }

    public void open() {

        trava.setPosition(0);
    }

    public void close() {
        trava.setPosition(0.50);

    }

}
