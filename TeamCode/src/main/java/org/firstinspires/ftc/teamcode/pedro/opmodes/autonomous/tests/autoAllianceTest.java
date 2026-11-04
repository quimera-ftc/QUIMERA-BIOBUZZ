package org.firstinspires.ftc.teamcode.pedro.opmodes.autonomous.tests;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class autoAllianceTest extends OpMode {


    PoseFactory poseFactory = PoseFactory.degrees();

    Pose startingPoint = poseFactory.of(86.06466512702079, 134.1431469720298, 0);
    Pose collect1 = poseFactory.of(130.97, 131.97, 0);
    Pose shoot2 = poseFactory.of(104.9, 113.2, 300); //(104.9, 113.2, 300)
    Pose collect2 = poseFactory.of(130.53, 95.13, 0); //(130.53, 95.13, 0)
    Pose shoot3 = poseFactory.of(102.93, 111.97, 330); //(102.93, 111.97, 330)
    Pose park = poseFactory.of(132.12685185185188, 51.289675925925924, 270);

    Path START_TO_COLLECT1() {
        return line(startingPoint, collect1).linear(startingPoint, collect1);
    }
    Path COLLECT1_TO_SHOOT2() {
        return line(collect1, shoot2).linear(collect1, shoot2);
    }
    Path SHOOT2_TO_COLLECT2() {
        return line(shoot2, collect2).linear(shoot2, collect2);
    }
    Path COLLECT2_TO_SHOOT3() {
        return line(collect2, shoot3).linear(collect2, shoot3);
    }
    Path SHOOT3_TO_PARK() {
        return line(shoot3, park).linear(shoot3, park);
    }

    public void loop(){

    }

    public void init() {


    }

    public void start() {

    }


}
