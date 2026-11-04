package org.firstinspires.ftc.teamcode.pedro.subsystems;

import androidx.annotation.NonNull;


import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.function.Supplier;

public class Turret{

    static final double maxPower = 1;
    double manualPower = 0;

    // --- mecanico ---
    DcMotorEx motor;
    final static double OFFSET_Y = -1.772;
    final static double OFFSET_X = 0.0;
    final double gearRatio = 50.0 / 15.0;
    final double ticksPerRev = 28.0 * (76.0 / 21.0) * gearRatio; // HD Hex = 28 ticks/volta; redução "4:1" é na real 76/21 (~3.62:1)
    final double ticksPerDegree = ticksPerRev / 360.0;
    static final double LIMIT_BUFFER = 90;
    static final double LIMIT_SPIN = 130;
    double lastError = 0, lastTime = 0;
    double integral = 0;
    static final double maxIntegral = 50;

    static final double angleTolerance = 2.0;
    final Constants.Alliance alliance;
    Pose currentPose;
    ElapsedTime timer = new ElapsedTime();
    double currentTime = 0;
    Mode mode = Mode.IDLE;

    public enum Mode {
        IDLE,
        TRACKING,
        MANUAL,
    }
    public Turret(@NonNull HardwareMap hardwareMap, @NonNull Pose startPose, @NonNull Constants.Alliance alliance) {
        motor = hardwareMap.get(DcMotorEx.class, "tower");

        currentPose = startPose;
        this.alliance = alliance;

        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motor.setDirection(DcMotorEx.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        timer.reset();

        initLimeLight(hardwareMap);
    }

    public void setManualPower(double power) {
        manualPower = power;
    }

    public void resetController() {
        integral = 0;
        lastError = 0;
    }

    public void update(Pose pose) {
        currentPose = pose;

        currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = dt;

        switch (mode) {

            case TRACKING:

                setPower(calculatePower(dt));
                break;

            case MANUAL:
                double currentAngle = getRawAngle();
                double safePower = manualPower;

                if (manualPower > 0) {
                    if (currentAngle % 360 >= LIMIT_SPIN) {
                        safePower = 0;
                    } else if (currentAngle % 360 > LIMIT_SPIN - LIMIT_BUFFER) {
                        double distanceToLimit = LIMIT_SPIN - (currentAngle % 360);
                        safePower = manualPower * (distanceToLimit / LIMIT_BUFFER);
                    }
                }
                else if (manualPower < 0) {
                    if (currentAngle % 360 <= -LIMIT_SPIN) {
                        safePower = 0;
                    } else if (currentAngle % 360 < -LIMIT_SPIN + LIMIT_BUFFER) {
                        double distanceToLimit = (currentAngle % 360) - (-LIMIT_SPIN);
                        safePower = manualPower * (distanceToLimit / LIMIT_BUFFER);
                    }
                }

                setPower(safePower);
                break;

            case IDLE:
            default:
                setPower(0);
                break;
        }
    }
    public void setMode(Mode newMode) {
        if (this.mode != Mode.TRACKING && newMode == Mode.TRACKING) {
            resetController();
        }
        this.mode = newMode;
    }
    public Mode getMode() {
        return mode;
    }
    public Pose getTrackingPose() {

        if (alliance == Constants.Alliance.BLUE) {

            if (currentPose.y() >= 70.75) {
                return Constants.topNestBLUE;
            } else return Constants.bottomNestBLUE;

        } else {

            if (currentPose.y() >= 70.75) {
                return Constants.topNestRED;
            } else return Constants.bottomNestRED;
        }

    }

    public Pose getTurretPose() {
        double heading = currentPose.heading();

        double rotated_offsetX = OFFSET_X * Math.cos(heading) - OFFSET_Y * Math.sin(heading);
        double rotated_offsetY = OFFSET_X * Math.sin(heading) + OFFSET_Y * Math.cos(heading);

        double turret_x = currentPose.x() + rotated_offsetX;
        double turret_y = currentPose.y() + rotated_offsetY;

        return new Pose(turret_x, turret_y);
    }

    public double getRawAngle() {
        return motor.getCurrentPosition() / ticksPerDegree;
    }
    public double getAbsoluteAngle() {
        return Math.toDegrees(currentPose.heading()) + getRawAngle();
    }


    public void resetEncoder() {
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }


    public double calculatePower(double dt) {
        double error = clampToLimits(computeTargetAngle() + getLimeLightOffset())- getRawAngle();

        integral += error * dt;
        integral = Range.clip(integral, -maxIntegral, maxIntegral);

        double derivative = (error - lastError) / dt;
        lastError = error;

        double pOut = Constants.TurretPIDCoefficients.kP * error;
        double iOut = Constants.TurretPIDCoefficients.kI * integral;
        double dOut = Constants.TurretPIDCoefficients.kD * derivative;

        double sOut = 0;
        if (Math.abs(error) > angleTolerance) {
            sOut = Math.signum(error) * Constants.TurretPIDCoefficients.kS;
        }

        return pOut + iOut + dOut + sOut;
    }

    private void setPower(double power) {
        motor.setPower(Range.clip(power, -maxPower, maxPower));
    }

    public double computeTargetAngle() {

        Pose turretPose = getTurretPose();
        Pose trackedPose = getTrackingPose();


        double globalDesiredAngle = Math.toDegrees(
                Math.atan2(trackedPose.y() - turretPose.y(), trackedPose.x() - turretPose.x())
        );


        double robotHeadingDeg = Math.toDegrees(currentPose.heading());

        double localDesiredAngle = globalDesiredAngle - robotHeadingDeg;

        double normalizedTarget = normalizeDegrees(localDesiredAngle);
        // Normaliza o alvo para mantê-lo entre -180 e 180
        return -Range.clip(normalizedTarget, -LIMIT_SPIN, LIMIT_SPIN);
    }

    public double clampToLimits(double angles) {
        return Range.clip(angles, -LIMIT_SPIN, LIMIT_SPIN);
    }

    public double normalizeDegrees(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public Command autoaim(Supplier<Pose> poseSupplier) {
        return Command.build()
                .setStart(() -> setMode(Mode.TRACKING))
                .setExecute(() -> update(poseSupplier.get()))
                .setEnd((endCondition) -> {
                    setMode(Mode.IDLE);
                    setPower(0);
                })
                .requiring(this);
    }



//-------------------laimilaite--------------------------------


    Limelight3A limelight;
    private final ElapsedTime timerLL = new ElapsedTime();

    private static final int LL_PIPELINE = 0;
    private double goalX = 0;
    private double angleToleranceLL = 0.5;
    private double kLL = 1.5;
    private static final double MAX_OFFSET_LL = 6.0;
    private double offsetLL = 0;


    public void initLimeLight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LL_PIPELINE);
        limelight.start();
        timerLL.reset();
    }

    public double getLimeLightOffset() {
        double dtLL = timerLL.seconds();
        timerLL.reset();

        if(limelight == null){
            return 0;
        }

        LLResult result = limelight.getLatestResult();

        if (result != null & result.isValid() && dtLL > 0) {
            double errorLL = result.getTx() - goalX;

            if (Math.abs(errorLL) > angleToleranceLL) {

                offsetLL += errorLL * kLL * Math.min(dtLL, 0.1);
                offsetLL = Range.clip(offsetLL, -MAX_OFFSET_LL, MAX_OFFSET_LL);
            }
        }
        return offsetLL;
    }

    public void resetLimelightOffset() {
        offsetLL = 0;
    }

    public void stopLimelight() {
        if (limelight != null) limelight.stop();
    }

}
