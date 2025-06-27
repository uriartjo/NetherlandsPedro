package pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "States All Bucket Auto - Pedro")
public class StatesBucketAuto extends LinearOpMode {

    private Follower follower;
    private Timer opmodeTimer, actionTimer;

    DcMotor viper1, viper2, viper3, pull, pull2;
    Servo rotation, parRotation, claw, clawRot;

    int fullExtend = 1600;
    int vert = -1150;

    // Servo positions
    double open = 0;
    double close = 0.35;
    double perp = 0.5;
    double par = 0.15;

    // Poses
    private Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private Pose scorePose = new Pose(4, 27, Math.toRadians(315));
    private Pose pickup1Pose = new Pose(27, 17, Math.toRadians(0));
    private Pose pickup2Pose = new Pose(23.5, 27, Math.toRadians(0));
    private Pose pickup3Pose = new Pose(37, 28.5, Math.toRadians(90));
    private Pose parkPose = new Pose(52, 0, Math.toRadians(270));

    // Paths
    private Path scoreToPickup1, pickup1ToScore, scoreToPickup2, pickup2ToScore,
            scoreToPickup3, pickup3ToScore, scoreToPark, initialToScore;

    // States
    private enum AutoState {
        INITIAL_SETUP,
        TO_SCORE_1,
        SCORE_1,
        TO_PICKUP_1,
        PICKUP_1,
        TO_SCORE_2,
        SCORE_2,
        TO_PICKUP_2,
        PICKUP_2,
        TO_SCORE_3,
        SCORE_3,
        TO_PICKUP_3,
        PICKUP_3,
        TO_SCORE_4,
        SCORE_4,
        TO_PARK,
        PARK,
        IDLE
    }

    private AutoState currentState = AutoState.INITIAL_SETUP;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        initializeHardware();

        // Initialize Pedro Pathing
        follower = new Follower(hardwareMap, LConstants.class, FConstants.class);
        follower.setStartingPose(startPose);

        // Initialize timers
        opmodeTimer = new Timer();
        actionTimer = new Timer();

        // Build paths
        buildPaths();

        // Set initial positions
        claw.setPosition(close);
        clawRot.setPosition(par);

        waitForStart();

        if (isStopRequested()) return;

        opmodeTimer.resetTimer();

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            autonomousPathUpdate();
            telemetry.addData("Current State", currentState);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }

    private void initializeHardware() {
        viper1 = hardwareMap.dcMotor.get("viper1");
        viper2 = hardwareMap.dcMotor.get("viper2");
        viper3 = hardwareMap.dcMotor.get("viper3");

        pull = hardwareMap.dcMotor.get("puller");
        pull2 = hardwareMap.dcMotor.get("puller2");
        claw = hardwareMap.servo.get("claw");
        clawRot = hardwareMap.servo.get("clawRot");
        rotation = hardwareMap.servo.get("rotation");
        parRotation = hardwareMap.servo.get("parRotation");

        viper1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pull.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pull2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        claw.setDirection(Servo.Direction.REVERSE);
        viper2.setDirection(DcMotor.Direction.REVERSE);
        pull2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void buildPaths() {
        // Initial move to score
        initialToScore = new Path(new BezierLine(
                new Point(startPose.getX(), startPose.getY(), Point.CARTESIAN),
                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN)
        ));
        initialToScore.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Score to pickup 1
        scoreToPickup1 = new Path(new BezierLine(
                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN),
                new Point(pickup1Pose.getX(), pickup1Pose.getY(), Point.CARTESIAN)
        ));
        scoreToPickup1.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading());

        // Pickup 1 to score
        pickup1ToScore = new Path(new BezierLine(
                new Point(pickup1Pose.getX(), pickup1Pose.getY(), Point.CARTESIAN),
                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN)
        ));
        pickup1ToScore.setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading());

        // Score to pickup 2
        scoreToPickup2 = new Path(new BezierLine(
                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN),
                new Point(pickup2Pose.getX(), pickup2Pose.getY(), Point.CARTESIAN)
        ));
        scoreToPickup2.setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading());

        // Pickup 2 to score
        pickup2ToScore = new Path(new BezierLine(
                new Point(pickup2Pose.getX(), pickup2Pose.getY(), Point.CARTESIAN),
                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN)
        ));
        pickup2ToScore.setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading());

        // Score to pickup 3
        scoreToPickup3 = new Path(new BezierLine(
                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN),
                new Point(pickup3Pose.getX(), pickup3Pose.getY(), Point.CARTESIAN)
        ));
        scoreToPickup3.setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading());

        // Pickup 3 to score
        pickup3ToScore = new Path(new BezierLine(
                new Point(pickup3Pose.getX(), pickup3Pose.getY(), Point.CARTESIAN),
                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN)
        ));
        pickup3ToScore.setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading());

        // Score to park
        scoreToPark = new Path(new BezierLine(
                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN),
                new Point(parkPose.getX(), parkPose.getY(), Point.CARTESIAN)
        ));
        scoreToPark.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    private void autonomousPathUpdate() {
        switch (currentState) {
            case INITIAL_SETUP:
                pull(vert);
                upRotation();
                follower.followPath(initialToScore);
                setPathState(AutoState.TO_SCORE_1);
                break;

            case TO_SCORE_1:
                if (!follower.isBusy()) {
                    vipers(fullExtend);
                    pull(vert + 50);
                    actionTimer.resetTimer();
                    setPathState(AutoState.SCORE_1);
                }
                break;

            case SCORE_1:
                if (actionTimer.getElapsedTime() > 0.3) {
                    scoreRotation();
                }
                if (actionTimer.getElapsedTime() > 0.7) {
                    claw.setPosition(open);
                }
                if (actionTimer.getElapsedTime() > 1.3) {
                    upRotation();
                }
                if (actionTimer.getElapsedTime() > 2.3) {
                    vipers(130);
                    pickupRotation();
                    pullDown(0);
                    follower.followPath(scoreToPickup1);
                    setPathState(AutoState.TO_PICKUP_1);
                }
                break;

            case TO_PICKUP_1:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(AutoState.PICKUP_1);
                }
                break;

            case PICKUP_1:
                if (actionTimer.getElapsedTime() > 0.5) {
                    claw.setPosition(close);
                }
                if (actionTimer.getElapsedTime() > 1.5) {
                    pull(vert + 50);
                    upRotation();
                    vipers(fullExtend);
                    follower.followPath(pickup1ToScore);
                    setPathState(AutoState.TO_SCORE_2);
                }
                break;

            case TO_SCORE_2:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(AutoState.SCORE_2);
                }
                break;

            case SCORE_2:
                if (actionTimer.getElapsedTime() > 0.5) {
                    scoreRotation();
                }
                if (actionTimer.getElapsedTime() > 0.7) {
                    claw.setPosition(open);
                }
                if (actionTimer.getElapsedTime() > 1.2) {
                    upRotation();
                }
                if (actionTimer.getElapsedTime() > 2.2) {
                    vipers(500);
                    pickupRotation();
                    pullDown(0);
                    follower.followPath(scoreToPickup2);
                    setPathState(AutoState.TO_PICKUP_2);
                }
                break;

            case TO_PICKUP_2:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(AutoState.PICKUP_2);
                }
                break;

            case PICKUP_2:
                if (actionTimer.getElapsedTime() > 0.5) {
                    claw.setPosition(close);
                }
                if (actionTimer.getElapsedTime() > 1.5) {
                    pull(vert);
                    upRotation();
                    vipers(fullExtend);
                    follower.followPath(pickup2ToScore);
                    setPathState(AutoState.TO_SCORE_3);
                }
                break;

            case TO_SCORE_3:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(AutoState.SCORE_3);
                }
                break;

            case SCORE_3:
                if (actionTimer.getElapsedTime() > 0.5) {
                    scoreRotation();
                }
                if (actionTimer.getElapsedTime() > 0.7) {
                    claw.setPosition(open);
                }
                if (actionTimer.getElapsedTime() > 1.2) {
                    upRotation();
                }
                if (actionTimer.getElapsedTime() > 2.2) {
                    vipers(250);
                    pickupRotation();
                    pullDown(0);
                    clawRot.setPosition(perp);
                    follower.followPath(scoreToPickup3);
                    setPathState(AutoState.TO_PICKUP_3);
                }
                break;

            case TO_PICKUP_3:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(AutoState.PICKUP_3);
                }
                break;

            case PICKUP_3:
                if (actionTimer.getElapsedTime() > 0.5) {
                    claw.setPosition(close);
                }
                if (actionTimer.getElapsedTime() > 1.5) {
                    pull(vert);
                    clawRot.setPosition(par);
                    upRotation();
                    vipers(fullExtend);
                    follower.followPath(pickup3ToScore);
                    setPathState(AutoState.TO_SCORE_4);
                }
                break;

            case TO_SCORE_4:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(AutoState.SCORE_4);
                }
                break;

            case SCORE_4:
                if (actionTimer.getElapsedTime() > 0.5) {
                    scoreRotation();
                }
                if (actionTimer.getElapsedTime() > 0.7) {
                    claw.setPosition(open);
                }
                if (actionTimer.getElapsedTime() > 1.2) {
                    upRotation();
                }
                if (actionTimer.getElapsedTime() > 2.2) {
                    vipers(300);
                    pickupRotation();
                    pullDown(0);
                    follower.followPath(scoreToPark);
                    setPathState(AutoState.TO_PARK);
                }
                break;

            case TO_PARK:
                if (!follower.isBusy()) {
                    setPathState(AutoState.PARK);
                }
                break;

            case PARK:
                setPathState(AutoState.IDLE);
                break;

            case IDLE:
                // Do nothing
                break;
        }
    }

    private void setPathState(AutoState state) {
        currentState = state;
    }

    // Hardware control methods
    public void pull(int x) {
        pull.setTargetPosition(x);
        pull2.setTargetPosition(x);
        pull.setPower(0.8);
        pull2.setPower(0.8);
        pull.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pull2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void pullDown(int x) {
        pull.setTargetPosition(x);
        pull2.setTargetPosition(x);
        pull.setPower(0.5);
        pull2.setPower(0.5);
        pull.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pull2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void vipers(int x) {
        viper1.setTargetPosition(x);
        viper2.setTargetPosition(x);
        viper3.setTargetPosition(x);
        viper1.setPower(1);
        viper2.setPower(1);
        viper3.setPower(1);
        viper1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void pickupRotation() {
        rotation.setPosition(0);
        parRotation.setPosition(1);
    }

    public void upRotation() {
        rotation.setPosition(0.4);
        parRotation.setPosition(0.6);
    }

    public void scoreRotation() {
        rotation.setPosition(0.5);
        parRotation.setPosition(0.5);
    }
}