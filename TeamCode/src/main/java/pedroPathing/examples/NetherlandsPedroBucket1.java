package pedroPathing.examples;



import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.examples.Config.IntakeSubsystem;

/**
 * Autonomous OpMode using Pedro Pathing library for FTC robotics
 *
 * Pedro Pathing is a path following library that allows robots to smoothly navigate
 * between waypoints using mathematical curves (Bezier curves) rather than simple
 * straight lines. This creates more natural, efficient robot movement.
 *
 * This autonomous routine:
 * 1. Scores a preloaded game piece in the high bucket
 * 2. Picks up and scores 3 additional samples from the field
 * 3. Parks in the observation zone for bonus points
 */
@Autonomous(name = "Pedro Bucket 1")
public class NetherlandsPedroBucket1 extends OpMode {

    // PEDRO PATHING CORE COMPONENTS
    private Follower follower;           // Main path following object - handles robot movement
    private Timer pathTimer;             // Times individual path segments
    private Timer actionTimer;           // Times non-movement actions (like claw operations)
    private Timer opmodeTimer;           // Times the entire autonomous period

    // STATE MANAGEMENT
    private int pathState;               // Current step in the autonomous sequence
    private boolean isActionStarted;     // Flag to track if timed actions have begun

    // ROBOT SUBSYSTEMS
    private IntakeSubsystem intake;      // Custom subsystem for claw/arm/lift control

    // FIELD POSITIONS
    // Pose = (x, y, heading) where heading is in radians
    // Coordinates are in inches, with (0,0) typically at field corner
    private Pose startPose = new Pose(9, 112, 0);                          // Starting position
    private Pose scorePose = new Pose(19, 124, Math.toRadians(315));       // High bucket scoring position
    private Pose pickup1Pose = new Pose(30, 121, 0);                       // First sample pickup
    private Pose pickup2Pose = new Pose(30, 132, 0);                       // Second sample pickup
    private Pose pickup3Pose = new Pose(45, 128, Math.toRadians(90));      // Third sample pickup
    private Pose parkPose = new Pose(70, 102, Math.toRadians(270));        // Final parking position
    private Pose parkControlPose = new Pose(77, 132, Math.toRadians(270)); // Control point for smooth parking curve

    // PATH CHAINS
    // PathChain objects contain the mathematical description of robot paths
    // Each path can include multiple segments and specify how the robot should move
    private PathChain scorePreload;      // Path to score the preloaded sample
    private PathChain pickupMark1;       // Path to first sample
    private PathChain scoreMark1;        // Path back to bucket with first sample
    private PathChain pickupMark2;       // Path to second sample
    private PathChain scoreMark2;        // Path back to bucket with second sample
    private PathChain pickupMark3;       // Path to third sample
    private PathChain scoreMark3;        // Path back to bucket with third sample
    private PathChain park;              // Path to parking zone

    /**
     * Builds all the paths the robot will follow during autonomous
     *
     * PathBuilder creates smooth Bezier curves between points rather than jerky straight lines.
     * setLinearHeadingInterpolation() smoothly rotates the robot as it moves between poses.
     */
    public void buildPaths() {
        // Path from starting position to scoring position
        scorePreload = follower.pathBuilder()
                .addBezierCurve(new Point(startPose), new Point(scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        // Path from scoring position to first sample
        pickupMark1 = follower.pathBuilder()
                .addBezierCurve(new Point(scorePose), new Point(pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        // Path from first sample back to scoring position
        scoreMark1 = follower.pathBuilder()
                .addBezierCurve(new Point(pickup1Pose), new Point(scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        // Path to second sample
        pickupMark2 = follower.pathBuilder()
                .addBezierCurve(new Point(scorePose), new Point(pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        // Path from second sample back to scoring position
        scoreMark2 = follower.pathBuilder()
                .addBezierCurve(new Point(pickup2Pose), new Point(scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        // Path to third sample
        pickupMark3 = follower.pathBuilder()
                .addBezierCurve(new Point(scorePose), new Point(pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        // Path from third sample back to scoring position
        scoreMark3 = follower.pathBuilder()
                .addBezierCurve(new Point(pickup3Pose), new Point(scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        // Path to parking zone (uses 3 points to create a smooth S-curve)
        park = follower.pathBuilder()
                .addBezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    /**
     * Updates the current path state and resets the path timer
     * This is used to transition between different phases of autonomous
     */
    public void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
        isActionStarted = false; // Reset action flag when changing states
    }

    /**
     * Main autonomous state machine
     *
     * This method is called continuously during autonomous and manages the sequence
     * of robot actions. Each case represents a different phase of the autonomous routine.
     *
     * The state machine pattern allows for:
     * - Sequential execution of complex actions
     * - Conditional branching based on sensors/timers/position
     * - Easy debugging and modification of autonomous sequences
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // START: Begin path to scoring position
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;

            case 1: // MOVING TO SCORE: Prepare mechanisms and score preload
                // Prepare scoring mechanisms when robot reaches Y position 115
                // This happens while the robot is still moving, improving efficiency
                if (follower.getPose().getY() > 115) {
                    intake.closeClaw();           // Secure the preloaded sample
                    intake.setClawMid();          // Position claw for scoring
                    intake.rotateVert(3650);      // Lift arm to scoring height
                    intake.vipers(3700);          // Extend linear slides
                }

                // When robot finishes moving to scoring position
                if (!follower.isBusy()) {
                    // Start timed scoring sequence
                    if (!isActionStarted) {
                        actionTimer.resetTimer();
                        isActionStarted = true;
                        intake.openClaw(); // Release the sample
                    }

                    // Wait 2 seconds for sample to drop, then move to first pickup
                    if (actionTimer.getElapsedTimeSeconds() > 2) {
                        follower.followPath(pickupMark1, true);
                        setPathState(2);
                    }
                }
                break;

            case 2: // MOVING TO FIRST SAMPLE: Pick up first sample
                if (!follower.isBusy()) {
                    // TODO: Add sample pickup logic here
                    // intake.lowerArm();
                    // intake.openClaw();
                    // intake.closeClaw(); (after positioning)

                    follower.followPath(scoreMark1, true);
                    setPathState(3);
                }
                break;

            case 3: // SCORE FIRST SAMPLE
                if (!follower.isBusy()) {
                    // TODO: Add scoring logic here
                    // intake.raiseArm();
                    // intake.openClaw();

                    follower.followPath(pickupMark2, true);
                    setPathState(4);
                }
                break;

            case 4: // MOVING TO SECOND SAMPLE
                if (!follower.isBusy()) {
                    // TODO: Add sample pickup logic here

                    follower.followPath(scoreMark2, true);
                    setPathState(5);
                }
                break;

            case 5: // SCORE SECOND SAMPLE
                if (!follower.isBusy()) {
                    // TODO: Add scoring logic here

                    follower.followPath(pickupMark3, true);
                    setPathState(6);
                }
                break;

            case 6: // MOVING TO THIRD SAMPLE
                if (!follower.isBusy()) {
                    // TODO: Add sample pickup logic here

                    follower.followPath(scoreMark3, true);
                    setPathState(7);
                }
                break;

            case 7: // SCORE THIRD SAMPLE
                if (!follower.isBusy()) {
                    // TODO: Add scoring logic here

                    follower.followPath(park, true);
                    setPathState(8);
                }
                break;

            case 8: // PARKING
                if (!follower.isBusy()) {
                    // TODO: Add level 1 ascent/hanging logic here
                    // This might involve extending a hanging mechanism

                    // Set to invalid state to stop the state machine
                    setPathState(-1);
                }
                break;

            default:
                // Do nothing - autonomous is complete
                break;
        }
    }

    /**
     * Initialize the OpMode
     * Called once when the OpMode is selected but before START is pressed
     */
    @Override
    public void init() {
        // Initialize all timers
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Set up Pedro Pathing constants (tuning parameters for path following)
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        // Initialize the path follower with hardware map
        // The Follower handles all the low-level motor control for path following

        // Initialize robot subsystems
        intake = new IntakeSubsystem(hardwareMap);

        // Tell the follower where the robot starts on the field
        follower.setStartingPose(startPose);

        // Build all the paths the robot will follow
        buildPaths();

        // Initialize state
        pathState = -1; // Will be set to 0 when START is pressed
    }

    /**
     * Main loop - called repeatedly during autonomous
     * This runs at ~50Hz (every 20ms) during autonomous period
     */
    @Override
    public void loop() {
        // Update the path follower - this calculates motor powers and sends commands
        // This MUST be called every loop for Pedro Pathing to work
        follower.update();

        // Run our autonomous state machine
        autonomousPathUpdate();

        // Send diagnostic information to the Driver Station
        telemetry.addData("Path State", pathState);
        telemetry.addData("Action Timer", "%.1f sec", actionTimer.getElapsedTimeSeconds());
        telemetry.addData("Robot Position", "X: %.1f, Y: %.1f",
                follower.getPose().getX(), follower.getPose().getY());
        telemetry.addData("Robot Heading", "%.1f degrees",
                Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.update();
    }

    /**
     * Called once when the START button is pressed
     * This begins the autonomous routine
     */
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0); // Begin the autonomous sequence
    }
}