package org.firstinspires.ftc.teamcode.pedro.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {
    //right = 1 = bolinha nao passa
    //left = 0 = bolinha passa nao colocar como 0 pra nenhum robo bater e quebrar
    Servo servoLocker;
    Servo flowerLeft;
    Servo flowerRight;
    public final double LOCKING = 1, DOWN_RIGHT = 0.90, UP_LEFT = 1;
    public final double UP_RIGHT = 0, DOWN_LEFT = 0.20;
    public final double NOT_LOCKING = 0.60;

    public Servos(HardwareMap hardwareMap) {
        servoLocker = hardwareMap.get(Servo.class, "locker");
        flowerLeft = hardwareMap.get(Servo.class, "flowerLeft");
        flowerRight = hardwareMap.get(Servo.class, "flowerRight");


        flowerLeft.setDirection(Servo.Direction.FORWARD);
        flowerRight.setDirection(Servo.Direction.FORWARD);

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

    public void agroUp() {
        flowerRight.setPosition(UP_RIGHT);
        flowerLeft.setPosition(UP_LEFT);
    }

    public void agroDown() {
        flowerRight.setPosition(DOWN_RIGHT);
        flowerLeft.setPosition(DOWN_LEFT);

    }

}
