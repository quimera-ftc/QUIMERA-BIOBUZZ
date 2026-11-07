package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {
    //right = 1 = bolinha nao passa
    //left = 0 = bolinha passa nao colocar como 0 pra nenhum robo bater e quebrar
    Servo servoLocker;
    public final double LOCKING = 1;
    public final double NOT_LOCKING = 0.60;

    public Servos(HardwareMap hardwareMap) {
        servoLocker = hardwareMap.get(Servo.class, "locker");

    }

    public void locking() {
        servoLocker.setPosition(LOCKING);
    }

    public void notLocking() {
        servoLocker.setPosition(NOT_LOCKING);
    }

    public double getPosition() {
        return servoLocker.getPosition();
    }

}
