package org.firstinspires.ftc.teamcode.pedroPathing;

import static android.os.SystemClock.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.NanoTimer;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "TestAuto", group = "Autonomous")
@Configurable // Panels
public class TestAuto  extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private Timer pathTimer, actionTimer,opmodeTimer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(48, 2, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {

        public static PathChain Path1;
        public static PathChain Path2;
        public static PathChain Path3;
        public static PathChain Path4;
        public static PathChain Path5;
        public static PathChain Path6;
        public static PathChain Path7;
        public static PathChain Path8;
        public static PathChain Path9;
        public static PathChain Path10;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 8.000), new Pose(48.000, 35.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 35.000), new Pose(36.000, 35.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(36.000, 35.000), new Pose(30.000, 35.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(30.000, 35.000), new Pose(18.000, 35.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.000, 35.000), new Pose(48.000, 0.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 8.000), new Pose(48.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 60.000), new Pose(36.000, 60.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(36.000, 60.000), new Pose(30.000, 60.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(30.000, 60.000), new Pose(18.000, 60.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.000, 60.000), new Pose(48.000, 0.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();
        }
    }




    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Paths.Path1);

                if (follower.getPose().getX()>46 &&follower.getPose().getX()<50 && follower.getPose().getY()>34 && follower.getPose().getY()<36 && follower.getVelocity().getMagnitude()<1) {
                    setPathState(2);
                }
                break;
            case 1:
                follower.followPath(Paths.Path2);
                if (follower.getPose().getX()>34 &&follower.getPose().getX()< 38 && follower.getPose().getY()>34 && follower.getPose().getY()<36 && follower.getVelocity().getMagnitude()<1) {
                    setPathState(2);
                }
                break;

            case 2:
                follower.followPath(Paths.Path3);

                if (follower.getPose().getX()>28 &&follower.getPose().getX()<32 && follower.getPose().getY()>34 && follower.getPose().getY()<36 && follower.getVelocity().getMagnitude()<1) {
                    setPathState(3);
                }
                break;
            case 3:
                follower.followPath(Paths.Path4);

                if (follower.getPose().getX()>16 &&follower.getPose().getX()<20 && follower.getPose().getY()>34 && follower.getPose().getY()<36 && follower.getVelocity().getMagnitude()<1) {
                    setPathState(4);
                }
                break;
            case 4:
                follower.followPath(Paths.Path5);
                if (follower.getPose().getX()>46 &&follower.getPose().getX()<50 && follower.getPose().getY()>0 && follower.getPose().getY()<5 && follower.getVelocity().getMagnitude()<1) {
                    setPathState(5);
                }
                break;
            case 5:
                follower.followPath(Paths.Path6);

                if (follower.getPose().getX()>46 &&follower.getPose().getX()<50 && follower.getPose().getY()>58 && follower.getPose().getY()<62 && follower.getVelocity().getMagnitude()<1) {
                    setPathState(6);
                }
                break;
            case 6:
                follower.followPath(Paths.Path7);
                if (follower.getPose().getX()>34 &&follower.getPose().getX()< 38 && follower.getPose().getY()>58 && follower.getPose().getY()<62 && follower.getVelocity().getMagnitude()<1) {
                    setPathState(7);
                }
                break;

            case 7:
                follower.followPath(Paths.Path8);

                if (follower.getPose().getX()>28 &&follower.getPose().getX()<32 && follower.getPose().getY()>58 && follower.getPose().getY()<62 && follower.getVelocity().getMagnitude()<1) {
                    setPathState(8);
                }
                break;
            case 8:
                follower.followPath(Paths.Path9);

                if (follower.getPose().getX()>16 &&follower.getPose().getX()<20 && follower.getPose().getY()>58 && follower.getPose().getY()<62 && follower.getVelocity().getMagnitude()<1) {
                    setPathState(9);
                }
                break;
            case 9:
                follower.followPath(Paths.Path10);
                if (follower.getPose().getX()>46 &&follower.getPose().getX()<50 && follower.getPose().getY()>0 && follower.getPose().getY()<5 && follower.getVelocity().getMagnitude()<1) {
                    setPathState(10);
                }
                break;
        }

        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
