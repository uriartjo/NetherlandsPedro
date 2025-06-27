package pedroPathing.examples.Config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {

    private final Servo claw, clawRot, rotation, parRotation;

    private final DcMotor viper1, viper2, viper3, puller, puller2;

    public IntakeSubsystem (HardwareMap hardwareMap) {
        viper1 = hardwareMap.dcMotor.get("viper1");
        viper2 = hardwareMap.dcMotor.get("viper2");
        viper3 = hardwareMap.dcMotor.get("viper3");
        puller = hardwareMap.dcMotor.get("puller");
        puller2 = hardwareMap.dcMotor.get("puller2");
        claw = hardwareMap.servo.get("claw");
        clawRot = hardwareMap.servo.get("clawRot");
        rotation = hardwareMap.servo.get("rotation");
        parRotation = hardwareMap.servo.get("parRotation");

        viper1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        puller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        puller2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        puller2.setDirection(DcMotor.Direction.REVERSE);
        viper2.setDirection(DcMotor.Direction.REVERSE);

        puller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        puller2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viper1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viper2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viper3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void closeClaw() {
        claw.setPosition(RobotConstants.closedClaw);
    }

    public void openClaw() {
        claw.setPosition(RobotConstants.closedClaw);
    }

    public void setClawRotPerp() {
        clawRot.setPosition(RobotConstants.clawRotPerp);
    }

    public void setClawRotPar() {
        clawRot.setPosition(RobotConstants.clawRotPar);
    }

    public void setClawDown() {
        rotation.setPosition(RobotConstants.rotationDown);
        parRotation.setPosition(RobotConstants.rotationDown);
    }

    public void setClawMid() {
        rotation.setPosition(RobotConstants.rotationMid);
        parRotation.setPosition(RobotConstants.rotationMid);
    }

    public void extendVipers() {
        viper1.setPower(1);
        viper2.setPower(1);
        viper2.setPower(1);
    }

    public void contractVipers() {
        viper1.setPower(1);
        viper2.setPower(1);
        viper3.setPower(1);
    }

    public void rotateVert(int x) {
        puller.setTargetPosition(x);
        puller2.setTargetPosition(x);
        puller.setPower(1);
        puller2.setPower(1);
        puller.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        puller2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void vipers(int x) {
        viper1.setTargetPosition(x);
        viper2.setTargetPosition(x);
        viper3.setTargetPosition(x);
        viper1.setPower(1);
        viper2.setPower(1);
        viper3.setPower(3);
        viper1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }



}
