package org.firstinspires.ftc.teamcode.pedro.opmodes.autonomous;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;

import com.pedropathing.paths.Path;

public class MainAutonomous {


    PoseFactory factory = PoseFactory.degrees();


    Pose start = factory.of(70,70,90);

    private Path score1() {
        return line(start, factory.of(10,20,10));
    }



}
