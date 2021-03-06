package org.firstinspires.ftc.teamcode.autonomousAndTeleops.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.robot.robotClasses.RobotMovement;


@Autonomous(name = "Run to Position with Odometry")
@Disabled
public class RunToPositionWithOdometry extends LinearOpMode {
    //    DO: change motors
    //Drive motors
    DcMotor frontRightMotor, backRightMotor, frontLeftMotor, backLeftMotor;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    //    DO: update countsperinch
    final double COUNTS_PER_INCH = 733;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot();
        waitForStart();
        initializeOdometry();

        RobotMovement movement = new RobotMovement(frontRightMotor,frontLeftMotor,backRightMotor,backLeftMotor,globalPositionUpdate,this);

//        BEGIN OP MODE HERE:



        movement.runToPositionAndStop(0,50,90,0.5,0.1,2,2);
        sleep(1000);
        movement.runToPositionAndStop(25,25,90,0.5,0.2,2,2);
        sleep(1000);
        movement.runToPositionAndStop(25,0,90,0.5,0.1,2,2);
        sleep(1000);
        movement.runToPositionAndStop(0,0,180,0.5,0.2,2,2);

        while (opModeIsActive()) {
            //Display Global (x, y, theta) coordinates
            displayTelemetry(telemetry,globalPositionUpdate);
        }
        //Stop the thread
        globalPositionUpdate.stop();

    }

    void initializeRobot(){
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
//        DO: update motor names
        frontRightMotor = hardwareMap.dcMotor.get("FRM");
        backRightMotor = hardwareMap.dcMotor.get("BRM");
        frontLeftMotor = hardwareMap.dcMotor.get("FLM");
        backLeftMotor = hardwareMap.dcMotor.get("BLM");

        //DO: change this according to where odometry is mounted
        verticalLeft = backLeftMotor;
        verticalRight = backRightMotor;
        horizontal = frontRightMotor;

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //DO: reverse motors
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
    }

    void initializeOdometry(){
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        //DO: set robot starting position
        globalPositionUpdate.setPosition(0, 0, 90);
        positionThread.start();

        //DO: reverse needed encoders
        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseLeftEncoder();
//        globalPositionUpdate.reverseNormalEncoder();

    }


    public static void displayTelemetry (Telemetry telemetry, OdometryGlobalCoordinatePosition globalPositionUpdate){
        telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate());
        telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate());
        telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());


        telemetry.addData("Vertical left encoder position", globalPositionUpdate.getVerticalLeftEncoderWheelPosition());
        telemetry.addData("Vertical right encoder position", globalPositionUpdate.getVerticalRightEncoderWheelPosition());
        telemetry.addData("horizontal encoder position", globalPositionUpdate.getNormalEncoderWheelPosition());
        telemetry.update();
    }
}
