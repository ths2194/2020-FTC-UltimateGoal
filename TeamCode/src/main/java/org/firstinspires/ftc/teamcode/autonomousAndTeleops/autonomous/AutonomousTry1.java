package org.firstinspires.ftc.teamcode.autonomousAndTeleops.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opModes.MyOpMode;
import org.firstinspires.ftc.teamcode.robot.odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.robot.robotClasses.RobotMovement;
import org.firstinspires.ftc.teamcode.robot.vision.TensorFlow;


@Autonomous(name = "AutonomousTry1")
public class AutonomousTry1 extends MyOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeWithOdometry();
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setTargetPosition(450);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        gripper.setPosition(0); //change to 0
        TensorFlow myTensorFlow = new TensorFlow(hardwareMap);
        RobotMovement movement = new RobotMovement(FRM,FLM,BRM,BLM,telemetry,globalPositionUpdate);
        shooterPush.setPosition(shooterPushOffPosition);

        while (false && !isStopRequested() && !(controller1.leftBumper() && controller1.rightBumper())) {
            controller1.update();
            if (controller1.dpadLeftOnce() || controller1.dpadRightOnce()) {
                redBlueMultiplier *= -1;
            }
            telemetry.addData("Field Side", redBlueMultiplier == 1? "Blue" : "Red");
            telemetry.update();
        }
        ReadWriteFile.writeFile(redBlueMultiplierFile, String.valueOf(redBlueMultiplier)); //saves current value to file

        telemetry.addData("Initialization:","complete");
        telemetry.update();

        waitForStart();
        globalPositionUpdate.setPosition(-62,48,0);

//        BEGIN OP MODE HERE:
        movement.runToPositionAndStop(-54,49,-20,0.4,0.4,2,2);
        double startTimeForDetection = getRuntime();
        int numOfRings = myTensorFlow.getNumberOfRings();

        while (numOfRings == 0 && getRuntime()-startTimeForDetection < 1.5) {
            numOfRings = myTensorFlow.getNumberOfRings();
        }

//        shootPins(movement);

        movement.pathFindToRestrictedDirectionNoStop(-5,48,-16,0.5,0.4,2,2);
        movement.runToPositionAndStop(-5,48,-16,0.5,1.7,0.5,0.5);

        double targetSpeed = 1095;

        ((DcMotorEx) upperShooter).setVelocity(targetSpeed);
        ((DcMotorEx) lowerShooter).setVelocity(targetSpeed);

        sleep(1000);

        while ( Math.abs(((DcMotorEx) upperShooter).getVelocity() - targetSpeed) > 10 || Math.abs(((DcMotorEx) lowerShooter).getVelocity() - targetSpeed) > 10) {
        }
        shooterPush.setPosition(shooterPushOnPosition);
        sleep(700);

        shooterPush.setPosition(shooterPushOffPosition);

        targetSpeed = 1085;

        ((DcMotorEx) upperShooter).setVelocity(targetSpeed);
        ((DcMotorEx) lowerShooter).setVelocity(targetSpeed);

        sleep (500);

        while ( Math.abs(((DcMotorEx) upperShooter).getVelocity() - targetSpeed) > 10 || Math.abs(((DcMotorEx) lowerShooter).getVelocity() - targetSpeed) > 10) {
        }

        shooterPush.setPosition(shooterPushOnPosition);

        sleep(700);

        ((DcMotorEx) upperShooter).setVelocity(0);
        ((DcMotorEx) lowerShooter).setVelocity(0);

        sleep(1000);

        shooterPush.setPosition(shooterPushOffPosition);

        intakeMotor.setPower(0.85);

        while (intakeMotor.getCurrentPosition() != intakeMotor.getTargetPosition()){
        }

        sleep(200);

        intakeMotor.setPower(0);

        targetSpeed = 1085;

        ((DcMotorEx) upperShooter).setVelocity(targetSpeed);
        ((DcMotorEx) lowerShooter).setVelocity(targetSpeed);

        sleep(1000);

        while ( Math.abs(((DcMotorEx) upperShooter).getVelocity() - targetSpeed) > 10 || Math.abs(((DcMotorEx) lowerShooter).getVelocity() - targetSpeed) > 10) {
        }
        shooterPush.setPosition(shooterPushOnPosition);
        sleep(700);
        ((DcMotorEx) upperShooter).setVelocity(0);
        ((DcMotorEx) lowerShooter).setVelocity(0);






        if (numOfRings == 0) {
            movement.runToPositionAndStop(-6, 58, 0, 0.7, 0.4, 2, 10);
        }
        else if (numOfRings == 4) {
            movement.runToPositionAndStop(42, 58, 0, 0.7, 0.4, 2, 10);
        }
        else if (numOfRings == 1){
            movement.runToPosition(16, 50, -10, 0.7, 0.4, 2, 10);
            movement.runToPositionAndStop(24, 45, -45, 0.7, 0.6, 2, 10);
        }
        sleep(100);
        gripper.setPosition(0.7);

        shooterPush.setPosition(shooterPushOffPosition);

        movement.runToPositionAndStop(10, 33, 0, 0.7, 0.4, 2, 10);


        while (opModeIsActive()) {
            //Display Global (x, y, theta) coordinates
            displayTelemetry(telemetry,globalPositionUpdate,Integer.toString(numOfRings));
        }
        //Stop the thread
        globalPositionUpdate.stop();

    }

    private void shootPins(RobotMovement movement){
        movement.pathFindToRestrictedDirectionNoStop(-5,44,-37,0.5,0.4,2,2);
        movement.runToPositionAndStop(-5,44,-38,0.4,1.7,1,0.5);

        double targetSpeed = 1135;

        ((DcMotorEx) upperShooter).setVelocity(targetSpeed);
        ((DcMotorEx) lowerShooter).setVelocity(targetSpeed);

        sleep(1000);

        while ( Math.abs(((DcMotorEx) upperShooter).getVelocity() - targetSpeed) > 10 || Math.abs(((DcMotorEx) lowerShooter).getVelocity() - targetSpeed) > 10) {
        }
        shooterPush.setPosition(shooterPushOnPosition);
        sleep(700);
        ((DcMotorEx) upperShooter).setVelocity(0);
        ((DcMotorEx) lowerShooter).setVelocity(0);

        sleep(1000);

        shooterPush.setPosition(shooterPushOffPosition);

//        intakeMotor.setPower(0.7);

        movement.runToPositionAndStop(-5,44,-33,0.4,1.7,1,0.5);

//        while (intakeMotor.getCurrentPosition() != intakeMotor.getTargetPosition()){
//        }
//        intakeMotor.setPower(0);

        sleep(200);

        targetSpeed = 1130;

        ((DcMotorEx) upperShooter).setVelocity(targetSpeed);
        ((DcMotorEx) lowerShooter).setVelocity(targetSpeed);

        sleep(1000);

        while ( Math.abs(((DcMotorEx) upperShooter).getVelocity() - targetSpeed) > 10 || Math.abs(((DcMotorEx) lowerShooter).getVelocity() - targetSpeed) > 10) {
        }
        shooterPush.setPosition(shooterPushOnPosition);
        sleep(700);
        ((DcMotorEx) upperShooter).setVelocity(0);
        ((DcMotorEx) lowerShooter).setVelocity(0);

        sleep(1000);

        shooterPush.setPosition(shooterPushOffPosition);

        intakeMotor.setPower(0.7);

        movement.runToPositionAndStop(-5,44,-31,0.4,1.7,1,0.5);

        while (intakeMotor.getCurrentPosition() != intakeMotor.getTargetPosition()){
        }
        intakeMotor.setPower(0);

        sleep(200);

        targetSpeed = 1120;

        ((DcMotorEx) upperShooter).setVelocity(targetSpeed);
        ((DcMotorEx) lowerShooter).setVelocity(targetSpeed);

        sleep(1000);

        while ( Math.abs(((DcMotorEx) upperShooter).getVelocity() - targetSpeed) > 10 || Math.abs(((DcMotorEx) lowerShooter).getVelocity() - targetSpeed) > 10) {
        }
        shooterPush.setPosition(shooterPushOnPosition);
        sleep(700);
        ((DcMotorEx) upperShooter).setVelocity(0);
        ((DcMotorEx) lowerShooter).setVelocity(0);
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
