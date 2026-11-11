package org.firstinspires.ftc.teamcode.pedro.subsystems;

import static org.firstinspires.ftc.teamcode.pedro.math.TurretController.clampToLimits;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.LIMIT_BUFFER;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.LIMIT_SPIN;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.LL_PIPELINE;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.MAX_OFFSET_LL;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.TOLERANCE;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.alpha;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.angleToleranceLL;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.goalX;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.kLL;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.maxIntegral;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.maxPower;
import static org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants.ticksPerDegree;

import androidx.annotation.NonNull;


import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.math.TurretController;
import org.firstinspires.ftc.teamcode.pedro.subsystems.subConstants.TurretConstants;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;
import java.util.function.Supplier;

public class Turret{
    double manualPower = 0;
    public final DcMotorEx motor;
    private double lastError = 0, lastTime = 0, lastFilteredDerivative = 0;
    private double integral = 0;
    private double zeroTicks = 0;
    private final Constants.Alliance alliance;
    private Pose currentPose;
    private final ElapsedTime timer = new ElapsedTime();
    private double currentTime = 0;
    private Mode mode = Mode.IDLE;
    private final int[] aprilTags;
    public enum Mode {
        IDLE,
        TRACKING,
        MANUAL,
    }
    public Turret(@NonNull HardwareMap hardwareMap, @NonNull Pose startPose, @NonNull Constants.Alliance alliance) {
        motor = hardwareMap.get(DcMotorEx.class, "tower");

        if (alliance == Constants.Alliance.BLUE) {
            aprilTags = TurretConstants.tagsBLUE;
        } else aprilTags = TurretConstants.tagsRED;

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

    public boolean aligned() {
        return Math.abs(getDistanceToTarget()) <= TOLERANCE;
    }

    public double getDt() {
        return currentTime - lastTime;
    }

    public void update(Pose pose) {
        currentPose = pose;

        currentTime = timer.seconds();
        double dt = (currentTime - lastTime);
        if (dt <= 0.002 || dt > 0.1) dt = 0.02;
        lastTime = currentTime;

        switch (mode) {

            case TRACKING:
                if (aligned()) {
                    setPower(0);
                } else {

                    double error = clampToLimits(getDistanceToTarget() + getLimeLightOffset());

                    integral += error * dt;
                    integral = Range.clip(integral, -maxIntegral, maxIntegral);
                    double derivative = (error - lastError) / dt;
                    double filteredDerivative = (alpha * derivative) + ((1.0 - alpha) * lastFilteredDerivative);

                    lastFilteredDerivative = filteredDerivative;
                    setPower(TurretController.calculatePower(integral, error, filteredDerivative));
                    lastError = error;
                }
                break;

            case MANUAL:
                double safePower = getSafePower();

                setPower(safePower);
                break;

            case IDLE:
            default:
                setPower(0);
                break;
        }
    }

    private double getSafePower() {
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
        return safePower;
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

    public double getDistanceToTarget() {
        double target = TurretController.computeTargetAngle(currentPose, getTrackingPose());
        return target - getRawAngle();
    }

    public Pose getTrackingPose() {

        if (alliance == Constants.Alliance.BLUE) {

            if (currentPose.y() >= 70.75) {
                return FieldPose.topNestBLUE;
            } else return FieldPose.bottomNestBLUE;

        } else {

            if (currentPose.y() >= 70.75) {
                return FieldPose.topNestRED;
            } else return FieldPose.bottomNestRED;
        }

    }

    public double getRawAngle() {
        return (motor.getCurrentPosition() - zeroTicks) / ticksPerDegree;
    }

    public void correct(double deg) {
        zeroTicks += (deg * ticksPerDegree);
    }

    public double getZeroTicks() {
        return zeroTicks;
    }

    public void resetEncoder() {
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setPower(double power) {
        motor.setPower(Range.clip(power, -maxPower, maxPower));
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
    public double offsetLL = 0;

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

        if (result != null && result.isValid() && dtLL > 0) {

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            for (LLResultTypes.FiducialResult res: fiducialResults) {
                if (!validAprilTag(res.getFiducialId())) {
                    return 0;
                }
            }

            double errorLL = result.getTx() - goalX;

            if (Math.abs(errorLL) > angleToleranceLL) {

                offsetLL += errorLL * kLL * Math.min(dtLL, 0.1);
                offsetLL = Range.clip(offsetLL, -MAX_OFFSET_LL, MAX_OFFSET_LL);
            }
        }
        return offsetLL;
    }

    public boolean validAprilTag(int id) {
        for (int tag: aprilTags) {
            if (tag == id) {
                return true;
            };
        }
        return  false;
    }

    public void resetLimelightOffset() {
        offsetLL = 0;
    }

    public void stopLimelight() {
        if (limelight != null) limelight.stop();
    }

}