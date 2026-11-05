package org.firstinspires.ftc.teamcode.pedro.math;



import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;


public class VoltageCompensator {
    HardwareMap hardwareMap;

    VoltageSensor bateria;
    public VoltageCompensator(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        bateria = hardwareMap.voltageSensor.iterator().next();
    }

    public static double compensate(HardwareMap hardwareMap,double power) {
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        double pOut = power * (Constants.VoltageConstants.NOMINAL_VOLTAGE / voltageSensor.getVoltage());
        return Range.clip(pOut, -1, 1);
    }

    public double getVoltage() {
        double minVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                minVoltage = Math.min(minVoltage, voltage);
            }
        }
        return minVoltage;
    }

    public double compensate(double power) {
        double pOut = power * (Constants.VoltageConstants.NOMINAL_VOLTAGE / getVoltage());
        return Range.clip(pOut, -1, 1);
    }

}
