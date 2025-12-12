package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.Arrays;
import java.util.List;

@Config
@TeleOp(name = "CTETeleop")
public class CTETeleop extends LinearOpMode {
    // Dc Motor dec
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor Intake = null;
    private DcMotor Indexer = null;
    private DcMotorEx Flywheel = null;

    private Servo leftKicker = null;
    private Servo rightKicker = null;

    private Servo Pitch = null;
    Limelight3A limelight;
    // Constants
    // indexer
    int rotationCounter = 360;
    int ballOne = 0;
    int ballTwo = ballOne + 180;
    int ballThree = ballTwo + 180;
    int[] ordera = { 2, 1, 1 };
    int[] orderb = { 1, 2, 1 };
    int[] orderc = { 1, 1, 2 };
    int[] targetOrder = orderc;
    int ballOneColor = 0;// 0 acts as null 1 is purple 2 is green
    int ballTwoColor = 0;// 0 acts as null 1 is purple 2 is green
    int ballThreeColor = 0;// 0 acts as null 1 is purple 2 is green

    boolean ballOneCounted = false;
    boolean ballTwoCounted = false;
    boolean ballThreeCounted = false;

    boolean sorting = false;

    int[] ballPosArray = { ballOne, ballTwo, ballThree };
    int[] ballColorArray = { ballOneColor, ballTwoColor, ballThreeColor };
    boolean lastintake = true;
    int segments = 0; // if moved too intake + 2 rotations moved to shooting + 1

    // cr servo
    private static final int rDistance = 0; // the change in encoder value between each slot
    // For normal servo mode
    private static final int Left = 0;
    private static final int Center = 0;
    private static final int Right = 0;
    // constants for indexer
    private ColorSensorV3 colorSensor;

    // declared like this so if motors are swapped I only have to find one value
    int TargetPosition = ballOne;
    int Intakepos = ballOne;
    int Shootingpos = ballOne + 270;
    private PIDControllerRyan indexerPID = null;
    private PIDTuner pidTuner = null;
    private FlywheelPIDController flywheelPID = null;

    // constants for Limlight
    // constants for Limlight
    private static final double WHEEL_RADIUS = .048; // meters (96mm diameter)
    private static final double SLIP_FACTOR = 1.0; // 1.0 = no slip

    // Ball properties
    private static final double BALL_RADIUS = .0635; // 5" diameter = 0.127m -> radius = 0.0635m
    private static final double BALL_AREA = Math.PI * BALL_RADIUS * BALL_RADIUS;
    public static final double HEIGHT_DIFF_METERS = 0.60; // Example: 60cm difference
    // Angle your limelight is mounted at (Degrees). 0 = pointing straight forward,
    // 20 = angled up.
    public static final double LL_MOUNT_ANGLE = 15.3;

    public static final double targetXOffset = 0.635; // distance from edge of hoop to center or best place ball to land
    // // includes distance from ll to shooting hole

    public static final double TargetY = .98425 - HEIGHT_DIFF_METERS;; // height of the collection zone - height of lime
    // light

    static double TargetX = 0;// find this

    double TargetAngle = 0;

    public static double VF = 0;
    public static double VA = 0;
    public static double VRPM = 0;
    int targetID = 0;
    double tx = 0;
    double ty = 0;
    boolean canSeeTarget = false;
    boolean isShooting = false;
    boolean aimedCorrectly = false;

    boolean green = false;
    boolean purple = false;

    @Override
    public void runOpMode() {

        // Lime Light
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        limelight.pipelineSwitch(0);
        // base
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        Indexer = hardwareMap.get(DcMotor.class, "Indexer");
        Flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        leftKicker = hardwareMap.get(Servo.class, "leftKicker");
        rightKicker = hardwareMap.get(Servo.class, "rightKicker");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        Indexer.setDirection(DcMotorSimple.Direction.FORWARD);
        Indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftKicker.setDirection(Servo.Direction.FORWARD);
        rightKicker.setDirection(Servo.Direction.FORWARD);
        Pitch = hardwareMap.get(Servo.class, "Pitch");
        leftKicker.setDirection(Servo.Direction.FORWARD);// 0 is min angle 1 is max angle
        setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // intake
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftKicker.setPosition(1);
        rightKicker.setPosition(0);
        // Color

        colorSensor = new ColorSensorV3(hardwareMap, telemetry, "colorSensor");
        // telemetry dec
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        indexerPID = new PIDControllerRyan(0.005, 0.0001, 0.0001, 0, Indexer);
        pidTuner = new PIDTuner(indexerPID, gamepad2, telemetry);
        flywheelPID = new FlywheelPIDController(0.0005, 0.00001, 0.00001, 0.0001);
        // threads
        Thread sortingThread = new Thread(this::SortingLoop);
        Thread LimeLightThread = new Thread(this::limeLightLoop);
        Thread colorThread = new Thread(this::color);
        waitForStart();
        sortingThread.start();
        colorThread.start();
        //LimeLightThread.start();

        while (opModeIsActive()) { // Loop
            colorSensor.addTelemetry();
            // --------------------------- WHEELS --------------------------- //
            // POV Mode uses left joystick to go forward & strafe, and right joystick to
            // rotate.
            double axial = Math.pow(-gamepad1.left_stick_y, 3); // Note: pushing stick forward gives negative value
            double lateral = Math.pow(gamepad1.left_stick_x, 3);
            double yaw = Math.pow(gamepad1.right_stick_x, 3);
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Intake
            if (gamepad1.left_trigger > 0.01) {
                Intake.setPower(1);
            } else {
                Intake.setPower(0);
            }
            if(gamepad1.right_trigger > 0.01){
                Flywheel.setPower(gamepad1.right_trigger);
            }
            else{
                Flywheel.setPower(0);
            }

            if (gamepad1.a) {
                if(ballOne == Shootingpos){
                    ballOneCounted = false;
                    ballOneColor = 0;
                }
                if(ballTwo == Shootingpos){
                    ballTwoCounted = false;
                    ballTwoColor = 0;
                }
                if(ballThree == Shootingpos){
                    ballThreeCounted = false;
                    ballThreeColor = 0;
                }
                Kick();
            }
            if (gamepad1.dpadUpWasPressed() && !sorting) {
                int temp = TargetPosition;
                if (lastintake) {
                    lastintake = false;
                    temp += 90;
                    ballOne += 90;
                    ballTwo = ballOne + 180;
                    ballThree = ballTwo + 180;
                } else {
                    temp += 180;
                    ballOne += 180;
                    ballTwo = ballOne + 180;
                    ballThree = ballTwo + 180;
                }
                if (temp >= rotationCounter) {
                    rotationCounter += 540;
                    Intakepos += 540;
                    Shootingpos += 540;
                }
                if (ballOne > Intakepos) {
                    Intakepos += 540;
                }
                TargetPosition = temp;
            }
            if (gamepad1.dpadRightWasPressed() && !sorting) {
                int temp = TargetPosition;
                if (!lastintake) {
                    lastintake = true;
                    temp += 90;
                    ballOne += 90;
                    ballTwo = ballOne + 180;
                    ballThree = ballTwo + 180;

                } else {
                    temp += 180;
                    ballOne += 180;
                    ballTwo = ballOne + 180;
                    ballThree = ballTwo + 180;
                }
                if (ballOne > Intakepos) {
                    Intakepos += 540;
                }
                if (temp >= rotationCounter) {
                    rotationCounter += 540;
                    Shootingpos += 540;
                }
                TargetPosition = temp;

            }

            /*if (gamepad1.bWasPressed()) {
                shootingLoop();
                if(Pitch.getPosition() * 60 != Math.toDegrees(TargetAngle)){
                    Pitch.setPosition(1);//60 degree shooting angle
                }
                else{
                    Pitch.setPosition(0);//40 degree shooting angle
                }
            }

             */

            if ((Intakepos == ballOne) && !ballOneCounted) {
                if (purple) {
                    ballOneColor = 1;
                    ballOneCounted = true;
                } else if (green) {
                    ballOneColor = 2;
                    ballOneCounted = true;
                } else {
                    ballOneColor = 0;
                }

            } else if ((Intakepos == ballTwo) && !ballTwoCounted) {
                if (purple) {
                    ballTwoColor = 1;
                    ballTwoCounted = true;
                } else if (green) {
                    ballTwoColor = 2;
                    ballTwoCounted = true;
                } else {
                    ballTwoColor = 0;
                }
            } else if ((Intakepos == ballThree) && !ballThreeCounted) {
                if (purple) {
                    ballThreeColor = 1;
                    ballThreeCounted = true;
                } else if (green) {
                    ballThreeColor = 2;
                    ballThreeCounted = true;
                } else {
                    ballThreeColor = 0;
                }
            }

            //if (tx < 8 && tx > -8) {
               // gamepad1.rumble(10);
            //}
           // else{
                //VRPM = 1500;
           // }
            PIDLoop();
            // --------------------------- TELEMETRY --------------------------- //
            // Show the elapsed game time and wheel power.

            telemetry.addData("Front left/Right", "%4.2f, %4.2f",
                    leftFrontDrive.getPower(), rightFrontDrive.getPower());
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f",
                    leftBackDrive.getPower(), rightBackDrive.getPower());
            telemetry.addData("Indexer Position", "%d",
                    Indexer.getCurrentPosition());
            telemetry.addData("Indexer Target Position", "%d",
                    TargetPosition);
            telemetry.addData("Target order", targetOrder);
            telemetry.addData("Ball one pos", ballOne);
            telemetry.addData("Ball two pos ", ballTwo);
            telemetry.addData("Ball three pos", ballThree);
            telemetry.addData("Ball one color", ballOneColor);
            telemetry.addData("Ball two color", ballTwoColor);
            telemetry.addData("Ball three color", ballThreeColor);
            telemetry.addData("green",green);
            telemetry.addData("purple",purple);
            telemetry.addData("Ball one counted", ballOneCounted);
            telemetry.addData("Ball two counted", ballTwoCounted);
            telemetry.addData("Ball three counted", ballThreeCounted);
            telemetry.addData("Shooting Position", Shootingpos);
            telemetry.addData("Intake Position", Intakepos);
            telemetry.addData("Current Order", targetOrder[0]);
            telemetry.addData("Current Order", targetOrder[1]);
            telemetry.addData("Current Order", targetOrder[2]);
            telemetry.addData("Flywheel Target RPM", VRPM);
            telemetry.addData("flywheel encoder",Flywheel.getCurrentPosition());
            telemetry.addData("distance", TargetX);
            telemetry.addData("ty", ty);
            telemetry.addData("tx", tx);
            telemetry.addData("ll status", limelight.isConnected());
            telemetry.addData("Flywheel Current RPM", Flywheel.getVelocity() * 60 / 28);
            telemetry.addData("Flywheel Power", Flywheel.getPower());
            telemetry.update();
        }
    }

    // Dedicated method for the PID loop

    private void PIDLoop() {
        /*
        double currentRPM = (Flywheel.getVelocity() / 28) / 60; // ticks per second to RPM
        double flypower = flywheelPID.update(VRPM, currentRPM);
        Flywheel.setPower(flypower);
         */
        // The PID controller calculates the necessary power to reach the TargetPosition
        double inpower = indexerPID.update(TargetPosition); // Use the D-pad updated TargetPosition
        Indexer.setPower(inpower);
        sleep(50);
    }

    private void SortingLoop() {
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.xWasPressed()) {
                sorting = true;
                int[] ballPosArray = { ballOne, ballTwo, ballThree };
                int[] ballColorArray = { ballOneColor, ballTwoColor, ballThreeColor };
                boolean arrived;
                for (int target : targetOrder) {
                    int i = 0;
                    arrived = false;
                    while (i < 3) {
                        if (ballColorArray[i] != target) {
                            i++;
                            continue;
                        }
                        if (!arrived) {
                            arrived = true;
                            int diff;
                            if (lastintake) {
                                if (ballPosArray[i] + 90 == Shootingpos) {
                                    diff = 90;
                                } else if (ballPosArray[i] + 270 == Shootingpos) {
                                    diff = 270;
                                } else {
                                    diff = 450;
                                }
                                lastintake = false;
                            } else {
                                if (ballPosArray[i] == Shootingpos) {
                                    diff = 0;
                                } else if (ballPosArray[i] + 180 == Shootingpos) {
                                    diff = 180;
                                } else {
                                    diff = 360;
                                }
                            }
                            int temp = TargetPosition + diff;
                            ballOne += diff;
                            ballTwo += diff;
                            ballThree += diff;
                            if (temp >= rotationCounter) {
                                rotationCounter += 540;
                                Shootingpos += 540;
                            }
                            if (ballOne > Intakepos) {
                                Intakepos += 540;
                            }
                            sleep(500);
                            TargetPosition = temp;
                        } else {
                            if (gamepad1.a) {
                                Kick();
                                if (ballColorArray[i] == ballOneColor) {
                                    ballOneColor = 0;
                                    ballOneCounted = false;
                                } else if (ballColorArray[i] == ballTwoColor) {
                                    ballTwoColor = 0;
                                    ballTwoCounted = false;
                                } else {
                                    ballThreeColor = 0;
                                    ballThreeCounted = false;
                                }

                                break;
                            } else if (gamepad1.y) {
                                break;

                            }
                        }
                    }
                }
            }
            sorting = false;
            sleep(20);
        }

    }
    private void color(){
        while(opModeIsActive()){
        if(colorSensor.isPurple()){
            purple = true;
            sleep(500);
        }
        else{
            purple = false;
        }
        if(colorSensor.isGreen()){
            green = true;
            sleep(500);
        }
        else {
            green = false;

        }
    }}
    // Dedicated method for the Limlight;

    private void limeLightLoop() {
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            tx = result.getTx();
            ty = result.getTy();


        }
        sleep(80);
    }
    private void shootingLoop() {
        if (tx < 5 && tx > -5) {
            Calculate();
            aimedCorrectly = true;
            gamepad1.rumble(10);
        }
    }
    public static double speedToRPM(double launchSpeed) {
        double wheelOmega = launchSpeed / (WHEEL_RADIUS * SLIP_FACTOR);
        // convert rad/s to RPM
        return wheelOmega * 60 / (2 * Math.PI);
    }

    public void Calculate() {

        // 1. CONVERT TO RADIANS
        double angleToGoalDegrees = LL_MOUNT_ANGLE + ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        // 2. CALCULATE DISTANCE (Horizontal X)
        double tanVal = Math.tan(angleToGoalRadians);
        if (Math.abs(tanVal) < 0.001)
            tanVal = 0.001;

        double distanceFromLimelight = (0.7493) / tanVal;

        // Add offset
        TargetX = distanceFromLimelight + targetXOffset;

        // 3. LAUNCH ANGLE
        // 60 degrees if > 2m, 40 degrees if closer
        double launchAngleDeg = (distanceFromLimelight > 2.0) ? 60.0 : 40.0;
        TargetAngle = Math.toRadians(launchAngleDeg);

        // 4. SOLVE VELOCITY
        solveShot(TargetX, TargetY, TargetAngle);
    }

    public static void solveShot(double xTarget, double yTarget, double launchAngle) {
        // Analytic solution for velocity (Vacuum model)
        // v = x / (cos(theta) * sqrt((2 * (x * tan(theta) - y)) / g))

        double g = 9.81;
        double cosTheta = Math.cos(launchAngle);
        double tanTheta = Math.tan(launchAngle);

        double term = xTarget * tanTheta - yTarget;

        double v = xTarget / (cosTheta * Math.sqrt((2.0 * term) / g));

        // Drag Compensation Factor (Heuristic)
        // Without simulation, we assume significant energy loss.
        v *= 1.15;

        VF = v;
        VA = launchAngle;
        double calculatedRPM = speedToRPM(v);
        VRPM = calculatedRPM;
    }

    private void Kick() {
        leftKicker.setPosition(0);
        rightKicker.setPosition(1);
        sleep(500);
        leftKicker.setPosition(1);
        rightKicker.setPosition(0);
    }

    private void setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }
}
