package org.firstinspires.ftc.teamcode.autonomousAndTeleops.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opModes.MyOpMode;
import org.firstinspires.ftc.teamcode.robot.odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.robot.robotClasses.RobotMovement;
import org.firstinspires.ftc.teamcode.robot.vision.TensorFlow;


@Autonomous(name = "DriveToCameraScan")
@Disabled
public class DriveToCameraScan extends MyOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeWithOdometry();
        gripper.setPosition(0);
        TensorFlow myTensorFlow = new TensorFlow(hardwareMap);
        RobotMovement movement = new RobotMovement(FRM,FLM,BRM,BLM,globalPositionUpdate,this);
        shooterPush.setPosition(0.4);

        telemetry.addData("Initialization:","complete");
        telemetry.update();

        waitForStart();
        globalPositionUpdate.setPosition(-62,48,0);

        globalPositionUpdate.stop();

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

    public static void displayTelemetry (Telemetry telemetry, OdometryGlobalCoordinatePosition globalPositionUpdate, String message){
        telemetry.addData("Message:",message);
        telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate());
        telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate());
        telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());


        telemetry.addData("Vertical left encoder position", globalPositionUpdate.getVerticalLeftEncoderWheelPosition());
        telemetry.addData("Vertical right encoder position", globalPositionUpdate.getVerticalRightEncoderWheelPosition());
        telemetry.addData("horizontal encoder position", globalPositionUpdate.getNormalEncoderWheelPosition());
        telemetry.update();
    }
}
