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
@TeleOp(name="TestTeleOp")
public class TestTeleOp extends LinearOpMode {
    //Dc Motor dec
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor Intake = null;
    private DcMotor Indexer = null;
    private DcMotorEx Flywheel = null;

    private Servo leftKicker =null;
    private Servo rightKicker =null;

    private Servo Pitch = null;
    Limelight3A limelight;
    //Constants
    //indexer
    int rotationCounter = 360;
    int ballOne = 0;
    int ballTwo = ballOne+180;
    int ballThree= ballTwo +180;
    int[] ordera = {2,1,1};
    int[] orderb = {1,2,1};
    int[] orderc = {1,1,2};
    int[] targetOrder = orderc;
    int ballOneColor = 0;// 0 acts as null 1 is purple 2 is green
    int ballTwoColor = 0;// 0 acts as null 1 is purple 2 is green
    int ballThreeColor = 0;// 0 acts as null 1 is purple 2 is green

    boolean ballOneCounted = false;
    boolean ballTwoCounted = false;
    boolean ballThreeCounted = false;

    int[] ballPosArray = {ballOne,ballTwo,ballThree};
    int[] ballColorArray = {ballOneColor,ballTwoColor,ballThreeColor};
    boolean lastintake = true;
    int segments = 0; //if moved too intake + 2 rotations moved to shooting + 1


    //cr servo
    private static final int rDistance = 0; //the change in encoder value between each slot
    //For normal servo mode
    private static final int Left = 0;
    private static final int Center = 0;
    private static final int Right = 0;
    //constants for indexer
    private ColorSensorV3 colorSensor;

    //declared like this so if motors are swapped I only have to find one value
    int TargetPosition = ballOne;
    int Intakepos = ballOne;
    int Shootingpos = ballOne + 270;
    private PIDControllerRyan indexerPID = null;
    private PIDTuner pidTuner = null;
    private FlywheelPIDController flywheelPID = null;

    //constants for Limlight
    private static final double WHEEL_RADIUS = .096;   // meters
    private static final double SLIP_FACTOR = 1.0;     // 1.0 = no slip

    // Ball properties
    private static final double BALL_RADIUS = .127;  // 5" diameter = 0.127m
    private static final double BALL_AREA = Math.PI * BALL_RADIUS * BALL_RADIUS;
    private static final double BALL_MASS = 0.20;      // kg — adjust for your ball
    // Air properties
    private static final double AIR_DENSITY = 1.225;   // kg/m^3
    private static final double DRAG_COEFF = 0.47;     // rough sphere
    private static final double LIFT_COEFF = 0.7;     // Magnus C_L (tunable)

    // Physics
    private static final double G = 9.81;

    // Simulation parameters
    private static final double DT = 0.002;            // time step (seconds)
    private static final double MAX_TIME = 5.0;

    public static final double LL_OFFSET = 15.3; //Limelight offset degrees
    public static final double HEIGHT_DIFF_METERS = 0.60; // Example: 60cm difference
    // Angle your limelight is mounted at (Degrees). 0 = pointing straight forward, 20 = angled up.
    public static final double LL_MOUNT_ANGLE = 15.3;

    public static final double targetXOffset = 0.3048; //distance from edge of hoop to center or best place ball to land // includes distance from ll to shooting hole

    public static final double TargetY = .98425 - HEIGHT_DIFF_METERS;; // height of the collection zone - height of lime light

    static double TargetX = 0;//find this

    double TargetAngle = 0;

    public static double VF =0;
    public static double VA = 0;
    public static double VRPM = 1000;

    private static final double llHeight = 260;
    private static final double flyWheelR = 96;
    int targetID = 24;// 24 is red //20 is blue
    //sorting order 21: g p p 22: p g p 23: p p g
    double tx = 0;
    double ty = 0;


    @Override
    public void runOpMode() {

        //Lime Light
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        limelight.pipelineSwitch(0);
        //base
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
        Flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftKicker.setDirection(Servo.Direction.FORWARD);
        rightKicker.setDirection(Servo.Direction.FORWARD);
        Pitch = hardwareMap.get(Servo.class, "Pitch");
        leftKicker.setDirection(Servo.Direction.FORWARD);// 0 is min angle 1 is max angle
        setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //intake
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftKicker.setPosition(1);
        rightKicker.setPosition(0);
        //Color
        colorSensor = new ColorSensorV3(hardwareMap, telemetry, "colorSensor");
        //telemetry dec
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        indexerPID = new PIDControllerRyan(0.005, 0.0001, 0.0001, 0, Indexer);
        pidTuner = new PIDTuner(indexerPID, gamepad2, telemetry);
        flywheelPID = new FlywheelPIDController(0.0005, 0.00001, 0.00001, 0.0001, Flywheel);
        //threads
        Thread indexerPIDThread = new Thread(this::indexerPIDLoop);
        Thread sortingThread = new Thread(this::SortingLoop);
        Thread LimeLightThread = new Thread(this::limeLightLoop);
        Thread flywheelPIDThread = new Thread(this::flywheelPIDLoop);
        waitForStart();
        indexerPIDThread.start();
        sortingThread.start();
        LimeLightThread.start();
        flywheelPIDThread.start();

        while (opModeIsActive()) { // Loop
            colorSensor.addTelemetry();
            //pidTuner.update();
            //debug
            // --------------------------- WHEELS --------------------------- //
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = Math.pow(-gamepad1.left_stick_y, 3);  // Note: pushing stick forward gives negative value
            double lateral = Math.pow(gamepad1.left_stick_x, 3);
            double yaw = Math.pow(gamepad1.right_stick_x, 3);
            double leftFrontPower =  axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower =  axial - lateral + yaw;
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

            // Reduced Power Mode
            if (gamepad1.a) {
                leftFrontPower /= 2;
                rightFrontPower /= 2;
                leftBackPower /= 2;
                rightBackPower /= 2;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            //Intake
            if(gamepad1.right_trigger > 0.01){
                Intake.setPower(1);
            }
            else{
                Intake.setPower(0);
            }

            if(gamepad1.a){
                Kick();
            }
            if(gamepad1.dpadUpWasPressed()){
                int temp = TargetPosition;
                if(lastintake){
                    lastintake = false;
                    temp += 90;
                    ballOne +=90;
                    ballTwo= ballOne + 180;
                    ballThree = ballTwo + 180;
                }
                else{
                    temp+=180;
                    ballOne+=180;
                    ballTwo = ballOne + 180;
                    ballThree = ballTwo + 180;
                }
                if(temp >= rotationCounter){
                    rotationCounter += 540;
                    Intakepos += 540;
                    Shootingpos += 540;
                }
                if(ballOne>Intakepos){
                    Intakepos += 540;
                }
                TargetPosition = temp;
            }
            if(gamepad1.dpadRightWasPressed()){
                int temp = TargetPosition;
                if(!lastintake){
                    lastintake = true;
                    temp += 90;
                    ballOne +=90;
                    ballTwo = ballOne + 180;
                    ballThree = ballTwo + 180;

                }
                else{
                    temp += 180;
                    ballOne += 180;
                    ballTwo= ballOne + 180;
                    ballThree = ballTwo + 180;
                }
                if(ballOne>Intakepos){
                    Intakepos += 540;
                }
                if(temp >= rotationCounter){
                    rotationCounter += 540;
                    Shootingpos += 540;
                }
                TargetPosition = temp;

            }
            if(gamepad1.bWasPressed()){
                Calculate();
            }
            if(Intakepos == ballOne && !ballOneCounted){
                if(colorSensor.isPurple()){
                    ballOneColor=1;
                    ballOneCounted = true;
                }
                else if(colorSensor.isGreen()){
                    ballOneColor=2;
                    ballOneCounted = true;
                }
                else{
                    ballOneColor=0;
                }
            }
            else if(Intakepos == ballTwo && !ballTwoCounted){
                if(colorSensor.isPurple()){
                    ballTwoColor=1;
                    ballTwoCounted = true;
                }
                else if(colorSensor.isGreen()){
                    ballTwoColor=2;
                    ballTwoCounted = true;
                }
                else{
                    ballTwoColor=0;
                }
            }
            else if(Intakepos == ballThree && !ballThreeCounted){
                if(colorSensor.isPurple()){
                    ballThreeColor=1;
                    ballThreeCounted = true;
                }
                else if(colorSensor.isGreen()){
                    ballThreeColor=2;
                    ballThreeCounted = true;
                }
                else{
                    ballThreeColor=0;
                }
            }


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
            telemetry.addData("Ball one counted", ballOneCounted);
            telemetry.addData("Ball two counted", ballTwoCounted);
            telemetry.addData("Ball three counted", ballThreeCounted);
            telemetry.addData("Shooting Position", Shootingpos);
            telemetry.addData("Intake Position", Intakepos);
            telemetry.addData("Current Order",targetOrder[0]);
            telemetry.addData("Current Order",targetOrder[1]);
            telemetry.addData("Current Order",targetOrder[2]);
            telemetry.addData("Flywheel Target RPM", VRPM);
            telemetry.addData("Flywheel Current RPM", -Flywheel.getVelocity() * 60 / 28);
            telemetry.addData("Flywheel Power", Flywheel.getPower());
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.update();
        }
    }

    // Dedicated method for the PID loop
    private void indexerPIDLoop() {
        while (opModeIsActive() && !isStopRequested()) { // Loop until OpMode stops
            // The PID controller calculates the necessary power to reach the TargetPosition
            double power = indexerPID.update(TargetPosition); // Use the D-pad updated TargetPosition
            Indexer.setPower(power);
            // Sleep to prevent the loop from hogging the CPU
            sleep(20); // Sleep for 20ms (a 50Hz loop is common for PID)
        }
    }
    private void flywheelPIDLoop() {
        while (opModeIsActive() && !isStopRequested()) {
                double currentRPM = Flywheel.getVelocity() * 60 / 28; // ticks per second to RPM
                double power = flywheelPID.update(VRPM, currentRPM);
                Flywheel.setPower(power);
            sleep(20);
        }
    }
    private void SortingLoop(){
        while(opModeIsActive() && !isStopRequested()){
            if(gamepad1.xWasPressed()){
                int[] ballPosArray = {ballOne,ballTwo,ballThree};
                int[] ballColorArray = {ballOneColor,ballTwoColor,ballThreeColor};
                boolean arrived;
                for(int target : targetOrder) {
                    int i = 0;
                    arrived = false;
                    while(i < 3) {
                        if(ballColorArray[i]!= target){
                            i++;
                            continue;
                        }
                        if (!arrived) {
                            arrived = true;
                            int diff;
                            if(lastintake) {
                                if (ballPosArray[i] + 90 == Shootingpos) {
                                    diff = 90;
                                } else if (ballPosArray[i] + 270 == Shootingpos) {
                                    diff = 270;
                                } else {
                                    diff = 450;
                                }
                                lastintake = false;
                            }
                            else{
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
                        }
                        else{
                            if (gamepad1.a) {
                                Kick();
                                if (ballColorArray[i] == ballOneColor){
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
                            }
                            else if(gamepad1.y){
                                break;

                            }
                        }
                    }
                }
            }
            sleep(50);
        }

    }
    // Dedicated method for the Limlight;

    private void limeLightLoop() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            if (id == targetID) {
                tx = fiducial.getTargetXDegrees(); // Where it is (left-right)
                ty = fiducial.getTargetYDegrees(); // Where it is (up-down)
                break;
            }
        }
        sleep(100);
        }

    public static double speedToRPM(double launchSpeed) {
        double wheelOmega = launchSpeed / (WHEEL_RADIUS * SLIP_FACTOR);
        return (wheelOmega * 60.0) / (2.0 * Math.PI);
    }
    public static Double simulateTrajectory(double v0, double theta,
                                            double spin, double targetX) {

        double vx = v0 * Math.cos(theta);
        double vy = v0 * Math.sin(theta);

        double x = 0;
        double y = 0;

        for (double t = 0; t < MAX_TIME; t += DT) {

            // If we passed target X, interpolate final height
            if (x >= targetX) {
                return y;
            }

            double v = Math.sqrt(vx * vx + vy * vy);

            // Drag force
            double drag = 0.5 * AIR_DENSITY * DRAG_COEFF * BALL_AREA * v;
            double dragX = drag * (vx / v);
            double dragY = drag * (vy / v);

            // Magnus lift force (spin × v) acts perpendicular to motion
            // Basic 2D approximation: lift acts +90° from velocity
            double lift = 0.5 * AIR_DENSITY * LIFT_COEFF * BALL_AREA * v * spin;

            double liftX = -lift * (vy / v);
            double liftY =  lift * (vx / v);

            // Accelerations
            double ax = (-dragX + liftX) / BALL_MASS;
            double ay = (-dragY + liftY) / BALL_MASS - G;

            // Integrate motion
            vx += ax * DT;
            vy += ay * DT;

            x += vx * DT;
            y += vy * DT;

            // Stop if ball hits ground early
            if (y < -1) return null;
        }

        return null;
    }
    public void Calculate() {
        if(VA == Math.toRadians(40)){
            Pitch.setPosition(1);
        }
        else{
            Pitch.setPosition(0);
        }
        // 1. CONVERT TO RADIANS
        // Limelight gives degrees, Java Math wants radians.
        double angleToGoalDegrees = LL_MOUNT_ANGLE + ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        // 2. CALCULATE DISTANCE (Horizontal X)
        // Tan(theta) = Opposite / Adjacent -> Adjacent = Opposite / Tan(theta)
        // Distance = HeightDiff / Tan(Angle)
        double distanceFromLimelight = (0.7493-HEIGHT_DIFF_METERS) / Math.tan(angleToGoalRadians);

        // Add offset if the goal center is deeper than the rim
        TargetX = distanceFromLimelight + targetXOffset;

        // 3. SOLVE
        // We want to hit a specific height (TargetY) relative to the shooter
        // Note: TargetY in solveShot should be the height of the goal relative to the shooter nozzle
        //spin is radians per second
        solveShot(TargetX, TargetY, -2);
    }

    public static void solveShot(double x, double yTarget, double spin) {
        double lowSpeed = 1.0;
        double highSpeed = 40.0;
        double bestSpeed = 0;
        double bestAngle = 0;
        int iterations = 0;

        // Binary search for the minimum SPEED required m/s
        while (highSpeed - lowSpeed > 0.01 && iterations < 100) {
            double midSpeed = (lowSpeed + highSpeed) / 2.0;

            // Try to find an angle that works for this speed
            Double angleResult = solveAngle(midSpeed, spin, x, yTarget);

            if (angleResult == null) {
                // This speed is too slow to reach the target even at optimal angle
                lowSpeed = midSpeed;
            } else {
                // This speed works! Let's see if we can go slower (usually better for accuracy)
                bestSpeed = midSpeed;
                bestAngle = angleResult;
                highSpeed = midSpeed;
            }
            iterations++;
        }

        VF = bestSpeed;
        VA = bestAngle;
        VRPM = speedToRPM(VF);
    }

    public static Double solveAngle(double v0, double spin, double x, double yTarget) {
        // Search range for launch angle (e.g., 10 to 60 degrees)
        double low = Math.toRadians(40);
        double high = Math.toRadians(55);
        double yTolerance = 0.1; // 10 cm tolerance

        Double yAtX;

        // Check high angle
        if (x > 2.1336) {
            yAtX = simulateTrajectory(v0, high, spin, x);
            if (yAtX != null && Math.abs(yAtX - yTarget) < yTolerance) {
                return high;
            }
        } else { // Check low angle
            yAtX = simulateTrajectory(v0, low, spin, x);
            if (yAtX != null && Math.abs(yAtX - yTarget) < yTolerance) {
                return low;
            }
        }

        return null; // No solution found for this speed
    }

    private void Kick(){
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
