package org.firstinspires.ftc.teamcode.pedro.robot;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.subsystems.Turret;

public class Robot {
    Intake intake;
    Turret turret;
    Shooter shooter;
    DriveTrain driveTrain;

    public Robot(@NonNull HardwareMap hardwareMap, @NonNull Constants.Alliance alliance) {
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap, PoseStorage.getPose(), alliance);
        shooter = new Shooter(hardwareMap);
//        driveTrain = new DriveTrain(hardwareMap);
    }
}
