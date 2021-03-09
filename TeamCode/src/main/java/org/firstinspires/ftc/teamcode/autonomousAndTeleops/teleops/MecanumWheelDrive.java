package org.firstinspires.ftc.teamcode.autonomousAndTeleops.teleops;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.rint;

@TeleOp(name = "MecanumWheelDrive", group = "TeleOp")

public class MecanumWheelDrive extends LinearOpMode {

    //Declare motor and servos ect.
    // make sure to import

    private DcMotor FRM;
    private DcMotor BRM;
    private DcMotor FLM;
    private DcMotor BLM;
    private DcMotor intakeMotor;
    private DcMotor upperShooter;
    private DcMotor lowewrShooter;
    private Servo gripper;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        FRM = hardwareMap.get(DcMotor.class, "FRM");
        BRM = hardwareMap.get(DcMotor.class, "BRM");
        FLM = hardwareMap.get(DcMotor.class, "FLM");
        BLM = hardwareMap.get(DcMotor.class, "BLM");
        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        upperShooter = hardwareMap.get(DcMotor.class, "UpperShooter");
        lowewrShooter = hardwareMap.get(DcMotor.class, "LowerShooter");
        gripper = hardwareMap.get(Servo.class, "gripper");

        FRM.setDirection(DcMotorSimple.Direction.REVERSE);
        BRM.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        upperShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lowewrShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upperShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lowewrShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        double directionMultiplier = 1;
        double intakeMotorPower = 0.5;
        double lastUpdate = 0;
        double targetShooterVelocity = 13000;

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                double FRPower, BRPower, FLPower, BLPower;

                FRPower = -gamepad1.left_stick_y - gamepad1.left_stick_x - 0.4*gamepad1.right_stick_x;
                BRPower = -gamepad1.left_stick_y + gamepad1.left_stick_x - 0.4*gamepad1.right_stick_x;
                FLPower = -gamepad1.left_stick_y + gamepad1.left_stick_x + 0.4*gamepad1.right_stick_x;
                BLPower = -gamepad1.left_stick_y - gamepad1.left_stick_x + 0.4*gamepad1.right_stick_x;

                double maxPower = max(max(max(abs(FRPower), abs(BRPower)), abs(FLPower)), abs(BLPower));
                FRPower /= maxPower;
                BRPower /= maxPower;
                FLPower /= maxPower;
                BLPower /= maxPower;

                // set power
                if (gamepad1.left_bumper) {
                    FLM.setPower(directionMultiplier * FLPower);
                    FRM.setPower(directionMultiplier * FRPower);
                    BLM.setPower(directionMultiplier * BLPower);
                    BRM.setPower(directionMultiplier * BRPower);

                } else if (gamepad1.left_trigger > 0) {
                    FLM.setPower(directionMultiplier * 0.2 * FLPower);
                    FRM.setPower(directionMultiplier * 0.2 * FRPower);
                    BLM.setPower(directionMultiplier * 0.2 * BLPower);
                    BRM.setPower(directionMultiplier * 0.2 * BRPower);
                }
                else {
                    FLM.setPower(directionMultiplier * 0.5 * FLPower);
                    FRM.setPower(directionMultiplier * 0.5 * FRPower);
                    BLM.setPower(directionMultiplier * 0.5 * BLPower);
                    BRM.setPower(directionMultiplier * 0.5 * BRPower);
                }

                if (gamepad1.a){
                    if (getRuntime() - lastUpdate > 0.2) {
                        if (Math.abs(intakeMotor.getPower()) > 0) {
                            intakeMotor.setPower(0);
                        } else {
                            intakeMotor.setPower(intakeMotorPower);
                        }
                        lastUpdate = getRuntime();
                    }
                }
                else if (gamepad1.y){
                    if (getRuntime() - lastUpdate > 0.2) {
                        if (Math.abs(intakeMotor.getPower()) > 0) {
                            intakeMotor.setPower(0);
                        } else {
                            intakeMotor.setPower(-intakeMotorPower);
                        }
                        lastUpdate = getRuntime();
                    }
                }

                if (gamepad1.dpad_up){
                    if (getRuntime() - lastUpdate > 0.2) {
                        intakeMotorPower = Math.min(intakeMotorPower+0.1,1);
                        if (Math.abs(intakeMotor.getPower()) > 0) {
                            intakeMotor.setPower(intakeMotorPower);
                        }
                        lastUpdate = getRuntime();
                    }
                }
                else if (gamepad1.dpad_down){
                    if (getRuntime() - lastUpdate > 0.2) {
                        intakeMotorPower = Math.max(intakeMotorPower-0.1,-1);
                        if (Math.abs(intakeMotor.getPower()) > 0) {
                            intakeMotor.setPower(intakeMotorPower);
                        }
                        lastUpdate = getRuntime();
                    }
                }

                if (gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0){
                    if (getRuntime() - lastUpdate > 0.2) {
                        directionMultiplier *= -1;
                        lastUpdate = getRuntime();
                    }
                }

                if (gamepad1.b) {
                    if (getRuntime() - lastUpdate > 0.2) {
                        if ( ((DcMotorEx) upperShooter).getVelocity() > 0) {
                            ((DcMotorEx) upperShooter).setVelocity(0);
                            ((DcMotorEx) lowewrShooter).setVelocity(0);
                        }
                        else{
                            ((DcMotorEx) upperShooter).setVelocity(targetShooterVelocity);
                            ((DcMotorEx) lowewrShooter).setVelocity(targetShooterVelocity);
                        }
                        lastUpdate = getRuntime();
                    }
                }

                if (gamepad1.dpad_right) {
                    targetShooterVelocity += 10;
                }
                else if (gamepad1.dpad_left) {
                    targetShooterVelocity -= 10;
                }

                if (gamepad1.x) {
                    if (getRuntime() - lastUpdate > 0.2) {
                        if (gripper.getPosition() != 1){
                            gripper.setPosition(1);
                        }
                        else {
                            gripper.setPosition(0);
                        }
                        lastUpdate = getRuntime();
                    }
                }



                telemetry.addData("FRPower", FRM.getPower());
                telemetry.addData("BRPower", BRM.getPower());
                telemetry.addData("FLPower", FLM.getPower());
                telemetry.addData("BLPower", BLM.getPower());
                telemetry.addData("IntakeMotorPower",intakeMotor.getPower());
                telemetry.addData("Target Shooter Velocity", targetShooterVelocity);
                telemetry.addData("Actual Shooter Velocity", ((DcMotorEx) upperShooter).getVelocity());

                telemetry.update();

            }
        }

    }

}

