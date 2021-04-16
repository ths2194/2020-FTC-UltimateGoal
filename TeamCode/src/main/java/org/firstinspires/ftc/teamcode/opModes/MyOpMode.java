package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.robot.odometry.OdometryGlobalCoordinatePosition;

import java.io.File;

public abstract class MyOpMode extends LinearOpMode {

//    public HardwareMap hardwareMap;
    private final double COUNTS_PER_INCH = 733;
    public OdometryGlobalCoordinatePosition globalPositionUpdate;

    public DcMotor FRM, BRM, FLM, BLM;
    public DcMotor intakeMotor;
    public DcMotor upperShooter, lowerShooter;
    public Servo gripper;
    public Servo shooterPush;
    public DcMotor verticalLeft, verticalRight, horizontal;

    public Controller controller1;
    public Controller controller2;

    public File redBlueMultiplierFile = AppUtil.getInstance().getSettingsFile("redBlueMultiplierFile.txt");
    public int redBlueMultiplier = Integer.parseInt(ReadWriteFile.readFile(redBlueMultiplierFile).trim());

    //constants
    public double shooterPushOffPosition = 0.05;
    public double shooterPushOnPosition = 0.23;


    public void initializeWithOdometry () {
        initializeRobot();
        initializeOdometry();
    }

    public void initializeRobot(){
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        FRM = hardwareMap.get(DcMotor.class, "FRM");
        BRM = hardwareMap.get(DcMotor.class, "BRM");
        FLM = hardwareMap.get(DcMotor.class, "FLM");
        BLM = hardwareMap.get(DcMotor.class, "BLM");

        //DO: change this according to where odometry is mounted
        verticalLeft = BLM;
        verticalRight = BRM;
        horizontal = FRM;

        FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        upperShooter = hardwareMap.get(DcMotor.class, "UpperShooter");
        lowerShooter = hardwareMap.get(DcMotor.class, "LowerShooter");
        gripper = hardwareMap.get(Servo.class, "gripper");
        shooterPush = hardwareMap.get(Servo.class, "ShooterPush");

        FRM.setDirection(DcMotorSimple.Direction.REVERSE);
        BRM.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        upperShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lowerShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upperShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowerShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initializeOdometry() {
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        //DO: set robot starting position
        positionThread.start();

        //DO: reverse needed encoders
        globalPositionUpdate.reverseRightEncoder();
//        globalPositionUpdate.reverseLeftEncoder();
//        globalPositionUpdate.reverseNormalEncoder();

    }

    public void setMotorPowers(double FLPower, double FRPower, double BLPower, double BRPower, double multiplier) {
        FLM.setPower(multiplier * FLPower);
        FRM.setPower(multiplier * FRPower);
        BLM.setPower(multiplier * BLPower);
        BRM.setPower(multiplier * BRPower);
    }

    public void setMotorPowers (double FLPower, double FRPower, double BLPower, double BRPower) {
        setMotorPowers(FLPower,FRPower,BLPower,BRPower,1);
    }

}
