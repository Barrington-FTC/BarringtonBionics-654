package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.IntakeLogic;
import org.firstinspires.ftc.teamcode.mechanisms.flyWheelLogic;
import org.firstinspires.ftc.teamcode.savedPosition;

@Autonomous(name = "red far auto", group = "Autonomous")
@Configurable // Panels
public class RFarAuto extends OpMode {

    private DcMotorEx Indexer = null;

    private DcMotorEx Flywheel = null;
    private DcMotorEx Turret = null;
    private Servo pitch = null;
    private Servo leftKicker = null;

    private flyWheelLogic shooter = new flyWheelLogic();

    private IntakeLogic intaker = new IntakeLogic();

    private boolean shotsTriggered = false;

    private PathConstraints constraints = new PathConstraints(1, .1, 1, .5, .5, .3, 1, .7);// so you can use the is busy
    // funtion not my bullshit
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private Timer pathTimer, actionTimer, opmodeTimer;
    private DigitalChannel laserInput;

    @Override
    public void init() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        // adds tollerances for when a path is considered complete
        constraints.setTranslationalConstraint(1);
        constraints.setVelocityConstraint(.1);
        constraints.setTimeoutConstraint(.5);
        constraints.setHeadingConstraint(.5);
        follower.setConstraints(constraints);
        follower.setStartingPose(new Pose(96, 0, Math.toRadians(90)));
        // indexer
        Indexer = hardwareMap.get(DcMotorEx.class, "Indexer");
        Indexer.setDirection(DcMotorSimple.Direction.FORWARD);
        Indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Indexer.setTargetPosition(0);
        Indexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Indexer.setPower(1);
        Indexer.setPositionPIDFCoefficients(10);

        // allows robot to do things in auto
        shooter.init(hardwareMap, Indexer);
        intaker.init(hardwareMap, Indexer);

        paths = new Paths(follower); // Build paths
        pitch = hardwareMap.get(Servo.class, "Pitch");
        Turret = hardwareMap.get(DcMotorEx.class, "turret");
        leftKicker = hardwareMap.get(Servo.class, "leftKicker");
        Flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");

        // flywheel
        Flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients flyhweelconts = new PIDFCoefficients(6.95,0,0,.7);
        Flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, flyhweelconts);
        // servos
        pitch.setPosition(0);
        leftKicker.setPosition(.01);
        // turret setup
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setTargetPosition(430);
        Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Indexer.setPositionPIDFCoefficients(18);
        Indexer.setTargetPositionTolerance(1);
        Turret.setPower(1);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        shooter.update();
        intaker.update();
        pathState = autonomousPathUpdate();// Update autonomous state machine

        // makes sure teleop gets position thats stopped on
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
        panelsTelemetry.debug("Power", shooter.getkickerpos());
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
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(96.000, 0.000), new Pose(96.000, 34.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(96.000, 35.000), new Pose(108.000, 34.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(108.000, 35.000), new Pose(113.000, 34.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(113.000, 35.000), new Pose(126.000, 34.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(126.000, 35.000), new Pose(96.000, 0.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(96.000, 8.000), new Pose(96.000, 40.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(96.000, 60.000), new Pose(108.000, 60.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(108.000, 60.000), new Pose(113.000, 60.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(113.000, 60.000), new Pose(126.000, 60.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(126.000, 60.000), new Pose(96.000, 8.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();
            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(96.000, 8.000), new Pose(96.000, 30.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:// preload
                if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                }
                if (!shooter.isBusy() && shooter.getShotsRemaning() == 0) {
                    shotsTriggered = false;
                    setPathState(1);
                }
                break;
            case 1:// in line with row 1
                follower.followPath(Paths.Path1);
                intaker.intakeBALLZ(1);
                if (followerArivved()) {
                    setPathState(2);
                }
                break;
            case 2:// r1 b1
                follower.followPath(Paths.Path2);
                intaker.intakeBALLZ(1);
                if (followerArivved()) {
                    setPathState(3);
                }
                break;

            case 3://r1 b2
                follower.followPath(Paths.Path3);
                intaker.intakeBALLZ(1);
                if (followerArivved()) {
                    setPathState(4);
                }
                break;
            case 4://r1 b3
                follower.followPath(Paths.Path4);
                if (followerArivved()) {
                    setPathState(5);
                    intaker.setNumBallz(0);
                }
                break;
            case 5://back to shoot pos
                follower.followPath(Paths.Path5);
                if (followerArivved()) {
                    setPathState(6);
                }
                if(pathTimer.getElapsedTimeSeconds()>15){
                    setPathState(7);
                }
                break;
            case 6://shoot 3 balls
                if (!shotsTriggered) {
                    shooter.fireShots(3);
                    shotsTriggered = true;
                }
                if (!shooter.isBusy() && shooter.getShotsRemaning() == 0) {
                    shotsTriggered = false;
                    setPathState(7);
                }
                if(pathTimer.getElapsedTimeSeconds()>10){
                    setPathState(7);
                }
                break;
                //these cases get skipped not enough time
            case 7://in line with r2
                follower.followPath(Paths.Path6);
                intaker.intakeBALLZ(1);
                if (followerArivved()) {
                    setPathState(13);//set to end in line with r2
                }
                break;
            case 8://r2 b1
                follower.followPath(Paths.Path7);
                intaker.intakeBALLZ(1);
                if (followerArivved()) {
                    setPathState(9);
                }
                break;

            case 9://r2 b2
                follower.followPath(Paths.Path8);
                intaker.intakeBALLZ(1);
                if (followerArivved()) {
                    setPathState(10);
                }
                break;
            case 10://r2 b3
                follower.followPath(Paths.Path9);
                if (followerArivved()) {
                    setPathState(11);
                    intaker.setNumBallz(0);
                }
                break;
            case 11:// back to shoot pos
                follower.followPath(Paths.Path10);
                if (followerArivved()) {
                    setPathState(12);
                }
                break;
            case 12://fire 3 balls
                if (!shotsTriggered) {
                    shooter.fireShots(3);
                    shotsTriggered = true;
                }
                if (!shooter.isBusy() && shooter.getShotsRemaning() == 0) {
                    shotsTriggered = false;
                    setPathState(7);
                }
                //path resumes
            case 13:// ranking points
                Indexer.setTargetPosition(0);
                if(!Indexer.isBusy()){
                terminateOpModeNow();}
        }
        return pathState;
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    private boolean followerArivved(){
        if((follower.getPose().getX()>follower.getCurrentPath().endPose().getX()-2 && follower.getPose().getX()<follower.getCurrentPath().endPose().getX()+2)&&(follower.getPose().getY()>follower.getCurrentPath().endPose().getY()-2 && follower.getPose().getY()<follower.getCurrentPath().endPose().getY()+2)&&(follower.getVelocity().getMagnitude()<1)){
            return true;
        }
        else{
            return false;
        }

    }

}
