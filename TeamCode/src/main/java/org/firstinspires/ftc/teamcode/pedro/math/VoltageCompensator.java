package org.firstinspires.ftc.teamcode.pedro.math;



import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;


public class VoltageCompensator {
    private final VoltageSensor voltageSensor;
    public VoltageCompensator(HardwareMap hardwareMap) {
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public static double compensate(HardwareMap hardwareMap,double power) {
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        double pOut = power * (Constants.VoltageConstants.NOMINAL_VOLTAGE / voltageSensor.getVoltage());
        return Range.clip(pOut, -1, 1);
    }

    public double compensate(double power) {
        double pOut = power * (Constants.VoltageConstants.NOMINAL_VOLTAGE / voltageSensor.getVoltage());
        return Range.clip(pOut, -1, 1);
    }

}
