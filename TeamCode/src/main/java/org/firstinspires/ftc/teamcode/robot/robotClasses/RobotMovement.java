package org.firstinspires.ftc.teamcode.robot.robotClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.odometry.OdometryGlobalCoordinatePosition;


public class RobotMovement {

    Telemetry telemetry;
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;
    OdometryGlobalCoordinatePosition globalPositionUpdate;

    public RobotMovement(DcMotor frontRightMotor, DcMotor frontLeftMotor, DcMotor backRightMotor, DcMotor backLeftMotor, Telemetry telemetry, OdometryGlobalCoordinatePosition globalPositionUpdate) {
        this.frontRightMotor = frontRightMotor;
        this.frontLeftMotor = frontLeftMotor;
        this.backRightMotor = backRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.telemetry = telemetry;
        this.globalPositionUpdate = globalPositionUpdate;
    }


    public void setTargetPowers (double targetX, double targetY, double desiredAngle, double robotPower, double turnSpeed) {
        //desiredAngle is in degrees

        double changeX = targetX - globalPositionUpdate.returnXCoordinate();
        double changeY = targetY - globalPositionUpdate.returnYCoordinate();

        double angleDifference = Math.atan2(changeY,changeX) - globalPositionUpdate.returnOrientationRadians();

        //changeY is the sideways change
        double relativeChangeX = Math.cos(angleDifference);
        double relativeChangeY = Math.sin(angleDifference);


        double pivotCorrection = Math.toRadians(desiredAngle) - globalPositionUpdate.returnOrientationRadians();
        // normalizes pivotCorrection to be between (-PI and PI)
        pivotCorrection = ((pivotCorrection % (2*Math.PI) ) + 3 * Math.PI) % (2*Math.PI) - Math.PI;
        //negative turn to right
        //positive turn to left

        double turnMovement = Range.clip(pivotCorrection / (Math.PI / 6), -1, 1 ) * turnSpeed;


        //finds largest power out of the 4 motor movement to normalize the values (make them all under 1)
        double largestPower =
                Math.max(Math.abs( relativeChangeX-relativeChangeY), Math.abs(relativeChangeX+relativeChangeY) );

        double frontLeftMotorPower = robotPower * (relativeChangeX-relativeChangeY) / largestPower - turnMovement;
        double frontRightMotorPower = robotPower * (relativeChangeX+relativeChangeY) / largestPower + turnMovement;
        double backLeftMotorPower = robotPower * (relativeChangeX+relativeChangeY) / largestPower - turnMovement;
        double backRightMotorPower = robotPower * (relativeChangeX-relativeChangeY) / largestPower + turnMovement;


        //finds largest power out of 4 motor turn and movement added to normalize values again (make them all under 1)
        largestPower =
                Math.max(Math.max(Math.max(Math.abs(frontLeftMotorPower), Math.abs(frontRightMotorPower)),Math.abs(backLeftMotorPower)),Math.abs(backRightMotorPower));

        if (largestPower>1) {
            frontLeftMotor.setPower(frontLeftMotorPower / largestPower);
            frontRightMotor.setPower(frontRightMotorPower / largestPower);
            backLeftMotor.setPower(backLeftMotorPower / largestPower);
            backRightMotor.setPower(backRightMotorPower / largestPower);
        }
        else{
            frontLeftMotor.setPower(frontLeftMotorPower);
            frontRightMotor.setPower(frontRightMotorPower);
            backLeftMotor.setPower(backLeftMotorPower);
            backRightMotor.setPower(backRightMotorPower);
        }

    }

    public void stop (){
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void runToPosition (double targetX, double targetY, double desiredAngle, double robotPower, double turnSpeed, double acceptableDistance, double acceptableAngle){

        //(Math.abs(desiredAngle - globalPositionUpdate.returnOrientation())+ acceptableAngle)%360 > acceptableAngle + acceptableAngle) this takes into account angles above or below within the acceptable angle range

        while ( Math.abs(targetX-globalPositionUpdate.returnXCoordinate()) > acceptableDistance || Math.abs(targetY-globalPositionUpdate.returnYCoordinate()) > acceptableDistance
                    || (Math.abs(desiredAngle - globalPositionUpdate.returnOrientation()) + acceptableAngle)%360 - acceptableAngle > acceptableAngle) {

            setTargetPowers(targetX,targetY,desiredAngle,robotPower,turnSpeed);
        }
        stop();
    }

    public void headTowardsPoint (double targetX, double targetY, double desiredHeading, double robotPower, double turnSpeed, double acceptableDistance) {
        // for desiredHeading, front of the robot is 0, back is 180, right side is 90, and left side is -90

        while ( Math.abs(targetX-globalPositionUpdate.returnXCoordinate()) > acceptableDistance || Math.abs(targetY-globalPositionUpdate.returnYCoordinate()) > acceptableDistance) {
            double desiredAngle = Math.toDegrees(Math.atan2(targetY - globalPositionUpdate.returnYCoordinate(),targetX - globalPositionUpdate.returnXCoordinate())) + desiredHeading;
            setTargetPowers(targetX,targetY,desiredAngle,robotPower,turnSpeed);
        }
        stop();
    }

}
