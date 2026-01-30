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
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.NanoTimer;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PIDControllerRyan;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeLogic;
import org.firstinspires.ftc.teamcode.mechanisms.flyWheelLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.savedPosition;

@Autonomous(name = "TestAuto", group = "Autonomous")
@Configurable // Panels
public class TestAuto  extends OpMode {

    private DcMotorEx Indexer = null;

    private DcMotorEx Flywheel = null;
    private DcMotorEx Turret = null;
    private Servo pitch = null;
    private Servo leftKicker = null;


    private flyWheelLogic shooter = new flyWheelLogic();

    private IntakeLogic intaker = new IntakeLogic();

    private boolean shotsTriggered = false;

    private PathConstraints constraints = new PathConstraints(1,.1,1,.5,.5,.3,1,.7);// so you can use the is busy funtion not my bullshit
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class


    private Timer pathTimer, actionTimer,opmodeTimer;



    @Override
    public void init() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        //adds tollerances for when a path is considered complete
        constraints.setTranslationalConstraint(1);
        constraints.setVelocityConstraint(.1);
        constraints.setTimeoutConstraint(.5);
        constraints.setHeadingConstraint(.5);
        follower.setConstraints(constraints);
        follower.setStartingPose(new Pose(48, 8, Math.toRadians(90)));
        //allows robot to do things in auto
        shooter.init(hardwareMap);
        intaker.init(hardwareMap);

        paths = new Paths(follower); // Build paths
        pitch = hardwareMap.get(Servo.class, "Pitch");
        Turret = hardwareMap.get(DcMotorEx.class, "turret");
        Indexer = hardwareMap.get(DcMotorEx.class, "Indexer");
        leftKicker = hardwareMap.get(Servo.class, "leftKicker");
        Flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");

        //flywheel
        Flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients flyhweelconts = new PIDFCoefficients(700,0,0,17);
        Flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,flyhweelconts);
        Flywheel.setVelocity(1250);
        //servos
        pitch.setPosition(1);
        leftKicker.setPosition(1);
        //turret setup
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setTargetPosition(600);
        Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Indexer.setPositionPIDFCoefficients(10);
        Turret.setPower(1);
        //indexer
        Indexer.setDirection(DcMotorSimple.Direction.FORWARD);
        Indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Indexer.setTargetPosition(0);
        Indexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Indexer.setPower(1);
        Indexer.setPositionPIDFCoefficients(10);


        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        shooter.update();
        shooter.pids();
        pathState = autonomousPathUpdate();// Update autonomous state machine

        //makes sure teleop gets position thats stopped on
        savedPosition.setX(follower.getPose().getX());
        savedPosition.sety(follower.getPose().getY());
        savedPosition.seth(follower.getPose().getHeading());

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Shooter State", shooter.getFlywheelState().toString());
        panelsTelemetry.debug("current pos", shooter.getIndexerPos());
        panelsTelemetry.debug("target pos", shooter.getIndexerTargetPos());
        panelsTelemetry.debug("Power", shooter.getIndexerpower());
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public static PathChain Path0;

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
        private static PathChain Path11;

        public Paths(Follower follower) {
            Path0 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 8.000), new Pose(48.000, 8.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();
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
                            new BezierLine(new Pose(18.000, 60.000), new Pose(48.000, 8.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();
            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 8.000), new Pose(48.000, 30.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();
        }
        }






    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0://preload
                if(!shotsTriggered) {
                    shooter.fireShots(3);
                    shotsTriggered = true;
                }
                if(!Indexer.isBusy() && shooter.getShotsRemaning()==0){
                    setPathState(1);
                }
                break;
            case 1:// in line with row 1
                follower.followPath(Paths.Path1);

                if (!follower.isBusy()) {
                    intaker.intakeBALLZ(1);
                    setPathState(2);
                }
                break;
            case 2://first ball pickup
                follower.followPath(Paths.Path2);
                if (!follower.isBusy()) {
                    setPathState(3);
                }
                break;

            case 3:
                follower.followPath(Paths.Path3);
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;
            case 4:
                follower.followPath(Paths.Path4);

                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;
            case 5:
                follower.followPath(Paths.Path5);
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;
            case 6:
                follower.followPath(Paths.Path6);

                if (!follower.isBusy()) {
                    setPathState(7);
                }
                break;
            case 7:
                follower.followPath(Paths.Path7);
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;

            case 8:
                follower.followPath(Paths.Path8);

                if (!follower.isBusy()) {
                    setPathState(9);
                }
                break;
            case 9:
                follower.followPath(Paths.Path9);

                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;
            case 10:
                follower.followPath(Paths.Path10);
                if (!follower.isBusy()) {
                    setPathState(11);
                }
                break;
            case 11:
                follower.followPath(Paths.Path11);
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
