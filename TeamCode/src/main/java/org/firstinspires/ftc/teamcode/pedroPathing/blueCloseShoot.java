package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Example Auto", group = "Examples")
public class blueCloseShoot extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        private int pathState;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(35.000, 135.000),

                                    new Pose(65.000, 79.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(125))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(65.000, 79.000),

                                    new Pose(65.000, 59.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }

        public void autonomousPathUpdate() {
            switch (pathState) {
                case 0:
                    follower.followPath(Path1,true);
            }

        }
    }
}