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

@TeleOp(name="NetherlandsTeleOp")
public class NetherlandsTeleOp extends OpMode {
    DcMotor viper1, viper2, viper3, puller, puller2;
    Servo claw, clawRot, rotation, parRotation;
    int hLimit = 2990;
    boolean overrideLimit = false;

    // PedroPathing follower instead of RoadRunner drive
    private Follower follower;

    @Override
    public void init() {
        viper1 = hardwareMap.dcMotor.get("viper1");
        viper2 = hardwareMap.dcMotor.get("viper2");
        viper3 = hardwareMap.dcMotor.get("viper3");
        puller = hardwareMap.dcMotor.get("puller");
        puller2 = hardwareMap.dcMotor.get("puller2");
        claw = hardwareMap.servo.get("claw");
        clawRot = hardwareMap.servo.get("clawRot");
        rotation = hardwareMap.servo.get("rotation");
        parRotation = hardwareMap.servo.get("parRotation");

        // Initialize PedroPathing follower with both constants classes
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(0, 0, 0));

        // parRotation.setDirection(Servo.Direction.REVERSE);
        viper1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        puller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        puller2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set drive motors to run without encoder (handled by PedroPathing)
        follower.setMaxPower(0.8); // Set max power for teleop

        puller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        puller2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viper1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viper2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viper3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        puller2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double lefty2 = gamepad2.left_stick_y;
        double leftx2 = gamepad2.left_stick_x;
        double righty2 = gamepad2.right_stick_y;
        double rightx2 = gamepad2.right_stick_x;
        boolean upOrDown = false;

        // Update follower (important for localization)
        follower.update();

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

        if(gamepad2.dpad_down) {
            rotation.setPosition(0);
            parRotation.setPosition(1);
        }
        if(gamepad2.dpad_left){
            rotation.setPosition(0.5);
            parRotation.setPosition(0.5);
        }

        if(gamepad2.dpad_up){
            rotation.setPosition(0.75);
            parRotation.setPosition(0.25);
        }

        if (righty2 <= -0.7) {
            if(puller.getCurrentPosition()<500) {
                // if (viper1.getCurrentPosition() < 3693 || viper2.getCurrentPosition() > -3734) {
                if (viper1.getCurrentPosition() < 3693 || viper2.getCurrentPosition() > -3734) {
                    viper1.setPower(1);
                    viper2.setPower(1);
                    viper3.setPower(1);
                }
                else {
                    viper1.setPower(0);
                    viper2.setPower(0);
                    viper3.setPower(0);
                }
            }else{
                if (viper1.getCurrentPosition() < 3700 || viper2.getCurrentPosition() > -3750) {
                    viper1.setPower(1);
                    viper2.setPower(1);
                    viper3.setPower(1);
                } else {
                    viper1.setPower(0);
                    viper2.setPower(0);
                    viper3.setPower(0);
                }
            }
        }
        if (righty2 >= 0.7) {
            viper1.setPower(-1);
            viper2.setPower(-1);
            viper3.setPower(-1);
        }

        if(gamepad2.b)
            claw.setPosition(0.65); // open claw
        if(gamepad2.a)
            claw.setPosition(1); // close claw
        if(gamepad2.x)
            clawRot.setPosition(0.15); // perpendicular
        if(gamepad2.y)
            clawRot.setPosition(0.5);
        if(gamepad2.right_bumper){
            clawRot.setPosition(1);
        }

        if(lefty2<-0.5){
            if(!overrideLimit) {
                if(puller.getCurrentPosition()<=3800) {
                    puller.setPower(0.7);
                    puller2.setPower(0.7);
                }
            } else {
                puller.setPower(0.7);
                puller2.setPower(0.7);
            }
        }
        if(lefty2>0.5){
            if(!overrideLimit) {
                if(puller.getCurrentPosition()>=0) {
                    puller.setPower(-0.4);
                    puller2.setPower(-0.4);
                }
            } else {
                puller.setPower(-0.4);
                puller2.setPower(-0.4);
            }
        }

        // PedroPathing teleop drive control
        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,   // forward/backward
                -gamepad1.left_stick_x,   // strafe left/right
                -gamepad1.right_stick_x   // rotation
        );

        telemetry.addData("encoder viper1:", viper1.getCurrentPosition());
        telemetry.addData("encoder viper2:", viper2.getCurrentPosition());
        telemetry.addData("encoder viper3:", viper3.getCurrentPosition());
        telemetry.addData("encoder puller:", puller.getCurrentPosition());
        telemetry.addData("Current Pose:", follower.getPose());
        telemetry.update();
    }

    public void pull(int x) {
        puller.setTargetPosition(x);
        puller.setPower(1);
        puller.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}