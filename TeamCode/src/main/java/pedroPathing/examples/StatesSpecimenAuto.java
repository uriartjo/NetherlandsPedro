package pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "Pedro States Specimen Auto")
public class StatesSpecimenAuto extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    DcMotor viper1, viper2, viper3, pull, pull2;
    Servo rotation, parRotation, claw, clawRot;

    int fullExtend = 1700;
    int vert = -1100;

    // Servo positions
    double open = 0;
    double close = 0.35;
    double perp = 0.5;
    double par = 0.15;

    // Path states
    private enum PathState {
        SCORE_PRELOAD,
        PICKUP_SAMPLE_1,
        PICKUP_SAMPLE_2,
        PICKUP_SAMPLE_3,
        PICKUP_HUMAN_1,
        SCORE_SPECIMEN_2,
        PICKUP_HUMAN_2,
        SCORE_SPECIMEN_3,
        PICKUP_HUMAN_3,
        SCORE_SPECIMEN_4,
        PICKUP_HUMAN_4,
        SCORE_SPECIMEN_5,
        PARK,
        IDLE
    }

    private PathState currentState = PathState.SCORE_PRELOAD;

    // Define all poses
    private Pose startPose = new Pose(0, 0, 0);
    private Pose scorePose = new Pose(-31.5, 0, 0);
    private Pose scorePose2 = new Pose(-31.5, -7, 0);
    private Pose scorePose3 = new Pose(-31.5, -3, 0);
    private Pose scorePose4 = new Pose(-31.5, -5, 0);
    private Pose scorePose5 = new Pose(-30, -1, 0);
    private Pose dropoffPose = new Pose(-10, 57, 0);
    private Pose dropoffPose2 = new Pose(-9, 55, 0);
    private Pose pickup1Pose = new Pose(-50, 30, 0);
    private Pose pickup2Pose = new Pose(-50, 42, 0);
    private Pose pickup3Pose = new Pose(-37, 51, Math.toRadians(90));
    private Pose humanPlayerPose = new Pose(-6, 28.5, 0);
    private Pose humanPlayerPose1 = new Pose(-15, 29.8, 0);
    private Pose humanPlayerPose2 = new Pose(-15, 30.5, 0);
    private Pose humanPlayerPose3 = new Pose(-9.5, 31.5, Math.toRadians(5));
    private Pose parkPose = new Pose(-9, 50, 0);

    // Define paths
    private Path scorePreloadPath;
    private PathChain sampleCollectionPath;
    private Path humanPlayer1Path;
    private Path scoreSpecimen2Path;
    private Path humanPlayer2Path;
    private Path scoreSpecimen3Path;
    private Path humanPlayer3Path;
    private Path scoreSpecimen4Path;
    private Path humanPlayer4Path;
    private Path scoreSpecimen5Path;
    private Path parkPath;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize follower
        follower = new Follower(hardwareMap, LConstants.class, FConstants.class);
        follower.setStartingPose(startPose);

        // Initialize hardware
        initializeHardware();

        // Build all paths
        buildPaths();

        // Initialize timers
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        // Set initial servo positions
        rotation.setPosition(0.5);
        parRotation.setPosition(0.4);
        clawRot.setPosition(0.9);
        claw.setPosition(close);

        waitForStart();

        if (isStopRequested()) return;

        // Start the autonomous sequence
        opmodeTimer.resetTimer();
        setPathState(PathState.SCORE_PRELOAD);

        while (opModeIsActive() && !isStopRequested()) {

            // Update follower
            follower.update();

            // Handle autonomous state machine
            autonomousPathUpdate();

            // Telemetry
            telemetry.addData("Path State", currentState);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }

    private void initializeHardware() {
        viper1 = hardwareMap.dcMotor.get("viper1");
        viper2 = hardwareMap.dcMotor.get("viper2");
        pull = hardwareMap.dcMotor.get("puller");
        pull2 = hardwareMap.dcMotor.get("puller2");
        claw = hardwareMap.servo.get("claw");
        clawRot = hardwareMap.servo.get("clawRot");
        rotation = hardwareMap.servo.get("rotation");
        parRotation = hardwareMap.servo.get("parRotation");

        viper1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pull.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pull2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        claw.setDirection(Servo.Direction.REVERSE);
        viper2.setDirection(DcMotor.Direction.REVERSE);
        pull2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void buildPaths() {
        // Score preload path
        scorePreloadPath = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreloadPath.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Sample collection path (complex multi-point path)
        sampleCollectionPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(-15, 28, Point.CARTESIAN), new Point(-50, 28, Point.CARTESIAN)))
                .addPath(new BezierLine(new Point(-50, 28, Point.CARTESIAN), new Point(-50, 45, Point.CARTESIAN)))
                .addPath(new BezierLine(new Point(-50, 45, Point.CARTESIAN), new Point(-12, 45, Point.CARTESIAN)))
                .addPath(new BezierLine(new Point(-12, 45, Point.CARTESIAN), new Point(-38, 35, Point.CARTESIAN)))
                .addPath(new BezierCurve(new Point(-38, 35, Point.CARTESIAN), new Point(-47, 52, Point.CARTESIAN)))
                .addPath(new BezierLine(new Point(-47, 52, Point.CARTESIAN), new Point(-11, 52, Point.CARTESIAN)))
                .build();

        // Human player paths
        humanPlayer1Path = new Path(new BezierLine(new Point(-11, 52, Point.CARTESIAN), new Point(humanPlayerPose)));
        humanPlayer1Path.setLinearHeadingInterpolation(0, humanPlayerPose.getHeading());

        humanPlayer2Path = new Path(new BezierLine(new Point(scorePose4), new Point(humanPlayerPose1)));
        humanPlayer2Path.setLinearHeadingInterpolation(scorePose4.getHeading(), humanPlayerPose1.getHeading());

        humanPlayer3Path = new Path(new BezierLine(new Point(scorePose2), new Point(humanPlayerPose2)));
        humanPlayer3Path.setLinearHeadingInterpolation(scorePose2.getHeading(), humanPlayerPose2.getHeading());

        humanPlayer4Path = new Path(new BezierLine(new Point(scorePose3), new Point(humanPlayerPose3)));
        humanPlayer4Path.setLinearHeadingInterpolation(scorePose3.getHeading(), humanPlayerPose3.getHeading());

        // Scoring paths
        scoreSpecimen2Path = new Path(new BezierLine(new Point(humanPlayerPose), new Point(scorePose4)));
        scoreSpecimen2Path.setLinearHeadingInterpolation(humanPlayerPose.getHeading(), scorePose4.getHeading());

        scoreSpecimen3Path = new Path(new BezierLine(new Point(humanPlayerPose1.getX() + 10, humanPlayerPose1.getY(), Point.CARTESIAN), new Point(scorePose2)));
        scoreSpecimen3Path.setLinearHeadingInterpolation(humanPlayerPose1.getHeading(), scorePose2.getHeading());

        scoreSpecimen4Path = new Path(new BezierLine(new Point(humanPlayerPose2.getX() + 10, humanPlayerPose2.getY(), Point.CARTESIAN), new Point(scorePose3)));
        scoreSpecimen4Path.setLinearHeadingInterpolation(humanPlayerPose2.getHeading(), scorePose3.getHeading());

        scoreSpecimen5Path = new Path(new BezierLine(new Point(humanPlayerPose3), new Point(scorePose5)));
        scoreSpecimen5Path.setLinearHeadingInterpolation(humanPlayerPose3.getHeading(), scorePose5.getHeading());

        // Park path
        parkPath = new Path(new BezierLine(new Point(scorePose3), new Point(parkPose)));
        parkPath.setLinearHeadingInterpolation(scorePose3.getHeading(), parkPose.getHeading());
    }

    private void autonomousPathUpdate() {
        switch (currentState) {
            case SCORE_PRELOAD:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    rotation.setPosition(0.5);
                    parRotation.setPosition(0.4);
                    clawRot.setPosition(0.9);
                    pull(vert);
                    claw.setPosition(close);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    speciPickupRotation();
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.4) {
                    claw.setPosition(close);
                    speciScoreRotation();
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.7) {
                    claw.setPosition(open);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.2) {
                    pullDown(0);
                    setPathState(PathState.PICKUP_SAMPLE_1);
                }
                break;

            case PICKUP_SAMPLE_1:
                if (!follower.isBusy()) {
                    setPathState(PathState.PICKUP_HUMAN_1);
                }
                break;

            case PICKUP_HUMAN_1:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    speciPickupRotation();
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    claw.setPosition(close);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    clawRot.setPosition(par);
                    pull(vert);
                    setPathState(PathState.SCORE_SPECIMEN_2);
                }
                break;

            case SCORE_SPECIMEN_2:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    speciScoreRotation();
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    claw.setPosition(open);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.2) {
                    pullDown(0);
                    clawRot.setPosition(0.9);
                    setPathState(PathState.PICKUP_HUMAN_2);
                }
                break;

            case PICKUP_HUMAN_2:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    speciPickupRotation();
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    // Move forward 10 inches (simulated by small delay)
                    if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                        claw.setPosition(close);
                    }
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.3) {
                    clawRot.setPosition(par);
                    pull(vert);
                    setPathState(PathState.SCORE_SPECIMEN_3);
                }
                break;

            case SCORE_SPECIMEN_3:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    speciScoreRotation();
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    claw.setPosition(open);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.2) {
                    pullDown(0);
                    clawRot.setPosition(0.9);
                    setPathState(PathState.PICKUP_HUMAN_3);
                }
                break;

            case PICKUP_HUMAN_3:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    speciPickupRotation();
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    // Move forward 10 inches (simulated by small delay)
                    if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                        claw.setPosition(close);
                    }
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.3) {
                    clawRot.setPosition(par);
                    pull(vert);
                    setPathState(PathState.SCORE_SPECIMEN_4);
                }
                break;

            case SCORE_SPECIMEN_4:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    speciScoreRotation();
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    claw.setPosition(open);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.8) {
                    setPathState(PathState.PARK);
                }
                break;

            case PARK:
                if (!follower.isBusy()) {
                    setPathState(PathState.IDLE);
                }
                break;

            case IDLE:
                break;
        }
    }

    private void setPathState(PathState newState) {
        currentState = newState;
        pathTimer.resetTimer();

        switch (newState) {
            case SCORE_PRELOAD:
                follower.followPath(scorePreloadPath);
                break;
            case PICKUP_SAMPLE_1:
                follower.followPath(sampleCollectionPath, true);
                break;
            case PICKUP_HUMAN_1:
                follower.followPath(humanPlayer1Path);
                break;
            case SCORE_SPECIMEN_2:
                follower.followPath(scoreSpecimen2Path);
                break;
            case PICKUP_HUMAN_2:
                follower.followPath(humanPlayer2Path);
                break;
            case SCORE_SPECIMEN_3:
                follower.followPath(scoreSpecimen3Path);
                break;
            case PICKUP_HUMAN_3:
                follower.followPath(humanPlayer3Path);
                break;
            case SCORE_SPECIMEN_4:
                follower.followPath(scoreSpecimen4Path);
                break;
            case PARK:
                follower.followPath(parkPath);
                break;
        }
    }

    // Hardware control methods
    public void pull(int x) {
        pull.setTargetPosition(x);
        pull2.setTargetPosition(x);
        pull.setPower(0.5);
        pull2.setPower(0.5);
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
        viper1.setPower(1);
        viper2.setPower(1);
        viper1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void speciScoreRotation() {
        rotation.setPosition(0.75);
        parRotation.setPosition(0.25);
    }

    public void speciPickupRotation() {
        rotation.setPosition(0.45);
        parRotation.setPosition(0.55);
    }
}