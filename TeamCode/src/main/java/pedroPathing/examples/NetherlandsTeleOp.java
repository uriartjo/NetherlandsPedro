package pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name="PedroPathing TeleOp")
public class NetherlandsTeleOp extends OpMode {
    DcMotor viper1, viper2, viper3, puller, puller2;
    Servo claw, clawRot, rotation, parRotation;
    int hLimit = 1500;
    long prevStepTime;
    boolean actionRunning, rotateUp, liftsUp, servoDown, clawOpen, liftsDown;
    boolean overrideLimit = false;
    boolean hLimitActivate = false;

    // PedroPathing drivetrain
    private Follower follower;

    @Override
    public void init() {
        // Initialize motors and servos
        viper1 = hardwareMap.dcMotor.get("viper1");
        viper2 = hardwareMap.dcMotor.get("viper2");
        viper3 = hardwareMap.dcMotor.get("viper3");
        puller = hardwareMap.dcMotor.get("puller");
        puller2 = hardwareMap.dcMotor.get("puller2");
        claw = hardwareMap.servo.get("claw");
        clawRot = hardwareMap.servo.get("clawRot");
        rotation = hardwareMap.servo.get("rotation");
        parRotation = hardwareMap.servo.get("parRotation");

        // Initialize PedroPathing Follower with your constants
        follower = new Follower(hardwareMap, LConstants.class, FConstants.class);

        // Set starting pose (adjust coordinates as needed for your field)
        follower.setStartingPose(new Pose(0, 0, 0));

        // Motor and servo configuration
        viper1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        puller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        puller2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        claw.setDirection(Servo.Direction.REVERSE);
        puller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        puller2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viper1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viper2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viper3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        puller2.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setPosition(0);
    }

    @Override
    public void start() {
        claw.setPosition(0.35);
    }

    @Override
    public void loop() {
        // Update follower (important for localization)
        follower.update();

        double lefty2 = gamepad2.left_stick_y;
        double leftx2 = gamepad2.left_stick_x;
        double righty2 = gamepad2.right_stick_y;
        double rightx2 = gamepad2.right_stick_x;
        boolean upOrDown = false;

        if(!actionRunning){
            puller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            puller2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            viper1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            viper2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            viper3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            viper1.setPower(0);
            viper2.setPower(0);
            viper3.setPower(0);
            puller.setPower(0);
            puller2.setPower(0);
        }

        // Height limit logic (commented out in original)
        // if(viper1.getCurrentPosition() >= hLimit && puller2.getCurrentPosition() >= -500) {
        //     hLimitActivate = true;
        // } else {
        //     hLimitActivate = false;
        // }

        if(!actionRunning) {
            // Servo controls
            if(gamepad2.dpad_down) {
                rotation.setPosition(0);
                parRotation.setPosition(1);
            }
            if(gamepad2.dpad_left){
                if(!hLimitActivate) {
                    rotation.setPosition(0.5);
                    parRotation.setPosition(0.5);
                }
            }

            if(gamepad2.dpad_up){
                if(!hLimitActivate) {
                    rotation.setPosition(0.75);
                    parRotation.setPosition(0.25);
                }
            }

            // Viper slide controls
            if (righty2 <= -0.7) {
                viper1.setPower(1);
                viper2.setPower(1);
                viper3.setPower(1);
            }

            if (righty2 >= 0.7) {
                viper1.setPower(-1);
                viper2.setPower(-1);
                viper3.setPower(-1);
            }

            // Claw controls
            if (gamepad2.b)
                claw.setPosition(0); // open claw
            if (gamepad2.a)
                claw.setPosition(0.35); // close claw
            if (gamepad2.y)
                clawRot.setPosition(0.15); // parallel
            if (gamepad2.x)
                clawRot.setPosition(0.5); // perp
        }

        // Specimen automation
        if(gamepad2.right_bumper){
            clawRot.setPosition(0.9);
        }

        // Additional servo control
        if(gamepad2.left_bumper){
            if(!hLimitActivate) {
                rotation.setPosition(0.45);
                parRotation.setPosition(0.55);
            }
        }

        // Puller controls
        if(lefty2 > 0.5){
            puller.setPower(0.5);
            puller2.setPower(0.5);
        }
        if(lefty2 < -0.5){
            puller.setPower(-0.9);
            puller2.setPower(-0.9);
        }

        // PedroPathing drivetrain control
        // Using teleop drive method for manual control
        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,    // forward/backward
                -gamepad1.left_stick_x,    // strafe left/right
                -gamepad1.right_stick_x,   // rotation
                true                       // field centric (set to false for robot centric)
        );

        // Telemetry
        telemetry.addData("encoder viper1:", viper1.getCurrentPosition());
        telemetry.addData("encoder viper2:", viper2.getCurrentPosition());
        telemetry.addData("encoder puller:", puller2.getCurrentPosition());
        telemetry.addData("hlimit activated:", hLimitActivate);

        // PedroPathing telemetry
        telemetry.addData("X Position:", follower.getPose().getX());
        telemetry.addData("Y Position:", follower.getPose().getY());
        telemetry.addData("Heading (Degrees):", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.update();
    }

    public void pull(int x) {
        puller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        puller2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        puller.setTargetPosition(x);
        puller2.setTargetPosition(x);

        puller.setPower(0.8);
        puller2.setPower(0.8);

        puller.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        puller2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
}