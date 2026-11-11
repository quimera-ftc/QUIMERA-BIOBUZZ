package org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretConstants {

    public static int[] tagsBLUE = {38, 39, 40, 41, 42, 43, 44, 45};
    public static int[] tagsRED = {32, 33, 31, 30, 34, 35, 36, 37};
    public static double MANUAL_POWER = .75;
    public static double CORRECTION_RATIO = 1;
    public static double OFFSET_Y = -1.772;
    public static double OFFSET_X = 0.000;
    public static double LIMIT_SPIN = 130;
    public static double maxIntegral = 50;
    public static double gearRatio = 50.0 / 15.0;
    public static double ticksPerRev = 28.0 * (76.0 / 21.0) * gearRatio; // HD Hex = 28 ticks/volta; redução "4:1" é na real 76/21 (~3.62:1)
    public static double ticksPerDegree = ticksPerRev / 360.0;
    public static double maxPower = 1;
    public static double LIMIT_BUFFER = 90;
    public static double TOLERANCE = 2.5;
    public static volatile double kP = 0.051721222;
    public static volatile double kI = 0.0000;
    public static volatile double kD = 0.00321222;
    public static volatile double kS = 0.25;
    public static volatile double alpha = 0.45; // entre 0 - 1
    public static double kV = 3.927; // rad/s
    // --------- limelight -----------
    public static int LL_PIPELINE = 0;
    public static double goalX = 0;
    public static double angleToleranceLL = 0.5;
    public static double kLL = 1.5;
    public static double MAX_OFFSET_LL = 6.0;

}