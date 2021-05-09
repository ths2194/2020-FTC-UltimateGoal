package org.firstinspires.ftc.teamcode.autonomousAndTeleops.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class AutonomousTry1 extends MyOpMode {

    boolean onLeft = true;
    TensorFlow myTensorFlow;
    RobotMovement movement;

    void waitForStartMenu() {
        int selection = 0;
        while (!isStopRequested() && !isStarted()) {
            controller1.update();
            if (controller1.dpadLeftOnce() || controller1.dpadRightOnce()) {
                if (selection == 0) onBlueSide = !onBlueSide;
                if (selection == 1) onLeft = !onLeft;
            }
            if (controller1.dpadDownOnce()) selection = (selection + 1) % 2;
            if (controller1.dpadUpOnce()) selection = (selection - 1 + 2) % 2;
            telemetry.addData("Initialization:","complete");
            telemetry.addData(selection == 0? "> Field Side" :  "  Field Side", onBlueSide? "Blue" : "Red");
            telemetry.addData(selection == 1? "> Start Spot" :  "  Start Spot", onLeft? "Left" : "Right");
            telemetry.update();
        }
        ReadWriteFile.writeFile(redBlueMultiplierFile, String.valueOf(onBlueSide)); //saves current value to file
    }

    int detectRings(){
        double startTimeForDetection = getRuntime();

        int numOfRings = myTensorFlow.getNumberOfRings();

        while (numOfRings == 0 && getRuntime()-startTimeForDetection < 1.5) {
            numOfRings = myTensorFlow.getNumberOfRings();
        }
        return numOfRings;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeWithOdometry();
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setTargetPosition(576);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        gripper.setPosition(0); //change to 0
        myTensorFlow = new TensorFlow(hardwareMap);
        movement = new RobotMovement(FRM,FLM,BRM,BLM,globalPositionUpdate,this);
        shooterPush.setPosition(shooterPushOffPosition);

        waitForStartMenu();

        int redBlueMultiplier = onBlueSide? 1 : -1;

//        waitForStart();
//        BEGIN OP MODE HERE:
        int numOfRings;
        if (onLeft) {
            globalPositionUpdate.setPosition(-62,48*redBlueMultiplier,0);
            movement.runToPositionAndStop(-54,48*redBlueMultiplier+1,-20,0.5,0.5,3,3);
            numOfRings = detectRings();
            movement.runToPosition(-4,48*redBlueMultiplier,-14,0.5,0.4,2,2);
            movement.runToPositionAndStop(-4,48*redBlueMultiplier,-14,0.5,1.7,0.5,0.5);

        }
        else {
            globalPositionUpdate.setPosition(-62,24*redBlueMultiplier,0);
            movement.runToPositionAndStop(-54,24*redBlueMultiplier-1,25,0.5,0.5,3,3);
            numOfRings = detectRings();
            movement.runToPosition(-4,24*redBlueMultiplier,7,0.5,0.4,2,2);
            movement.runToPositionAndStop(-4,24*redBlueMultiplier,7,0.5,1.7,0.5,0.5);

        }

//
//        movement.runToPosition(-6,48*redBlueMultiplier,0*redBlueMultiplier,0.5,0.4,5,5);
//
//        movement.runToPositionAndStop(-2,18*redBlueMultiplier,0*redBlueMultiplier,0.7,1.7,0.25,0.25);
//
//
//        double targetSpeed = 1065;
//
//        ((DcMotorEx) upperShooter).setVelocity(targetSpeed);
//        ((DcMotorEx) lowerShooter).setVelocity(targetSpeed);
//
//        sleep(1000);
//
//        while ( Math.abs(((DcMotorEx) upperShooter).getVelocity() - targetSpeed) > 10 || Math.abs(((DcMotorEx) lowerShooter).getVelocity() - targetSpeed) > 10) {
//        }
//        shooterPush.setPosition(shooterPushOnPosition);
//        sleep(500);
//
//        shooterPush.setPosition(shooterPushOffPosition);
//
//        ((DcMotorEx) upperShooter).setVelocity(0);
//        ((DcMotorEx) lowerShooter).setVelocity(0);
//
//        movement.runToPositionAndStop(-2,10*redBlueMultiplier,0*redBlueMultiplier,0.7,1.7,0.25,0.25);
//
//        targetSpeed = 1045;
//
//        ((DcMotorEx) upperShooter).setVelocity(targetSpeed);
//        ((DcMotorEx) lowerShooter).setVelocity(targetSpeed);
//
//        sleep (500);
//
//        while ( Math.abs(((DcMotorEx) upperShooter).getVelocity() - targetSpeed) > 10 || Math.abs(((DcMotorEx) lowerShooter).getVelocity() - targetSpeed) > 10) {
//        }
//
//        shooterPush.setPosition(shooterPushOnPosition);
//
//        sleep(500);
//
//        ((DcMotorEx) upperShooter).setVelocity(0);
//        ((DcMotorEx) lowerShooter).setVelocity(0);
//
//        sleep(1000);
//
//        shooterPush.setPosition(shooterPushOffPosition);
//
//        intakeMotor.setPower(0.9);
//
//        movement.runToPositionAndStop(-2,4*redBlueMultiplier,0*redBlueMultiplier,0.7,1.7,0.25,0.25);
//
//        while (intakeMotor.getCurrentPosition() != intakeMotor.getTargetPosition()){
//        }
//
//        sleep(200);
//
//        intakeMotor.setPower(0);
//
//        targetSpeed = 1045;
//
//        ((DcMotorEx) upperShooter).setVelocity(targetSpeed);
//        ((DcMotorEx) lowerShooter).setVelocity(targetSpeed);
//
//        sleep(1000);
//
//        while ( Math.abs(((DcMotorEx) upperShooter).getVelocity() - targetSpeed) > 10 || Math.abs(((DcMotorEx) lowerShooter).getVelocity() - targetSpeed) > 10) {
//        }
//        shooterPush.setPosition(shooterPushOnPosition);
//        sleep(300);
//        ((DcMotorEx) upperShooter).setVelocity(0);
//        ((DcMotorEx) lowerShooter).setVelocity(0);



        if (numOfRings == 0) {
            movement.runToPositionAndStop(-6, 58*redBlueMultiplier, 0*redBlueMultiplier, 0.7, 0.4, 2, 10);
        }
        else if (numOfRings == 4) {
            movement.runToPositionAndStop(42, 58*redBlueMultiplier, 0*redBlueMultiplier, 0.7, 0.4, 2, 10);
        }
        else if (numOfRings == 1){
            movement.runToPosition(16, 50*redBlueMultiplier, -10*redBlueMultiplier, 0.7, 0.4, 2, 10);
            movement.runToPositionAndStop(24, 45*redBlueMultiplier, -45*redBlueMultiplier, 0.7, 0.6, 2, 10);
        }
        sleep(100);
        gripper.setPosition(0.7);

        shooterPush.setPosition(shooterPushOffPosition);

        movement.runToPositionAndStop(10, 29*redBlueMultiplier, 0*redBlueMultiplier, 0.7, 0.4, 2, 10);

        ReadWriteFile.writeFile(endPosition, globalPositionUpdate.returnXCoordinate() + " " + globalPositionUpdate.returnYCoordinate() + " " + globalPositionUpdate.returnOrientation());

        while (opModeIsActive()) {
            //Display Global (x, y, theta) coordinates
            displayTelemetry(telemetry,globalPositionUpdate,Integer.toString(numOfRings));
        }
        //Stop the thread
        globalPositionUpdate.stop();

    }

    private void shootHighGoal (int redBlueMultiplier) {
        movement.pathFindToRestrictedDirectionNoStop(-4,48*redBlueMultiplier,-14*redBlueMultiplier,0.5,0.4,2,2);
        movement.runToPositionAndStop(-4,48*redBlueMultiplier,-14*redBlueMultiplier,0.5,1.7,0.5,0.5);

        double targetSpeed = 1095;

        ((DcMotorEx) upperShooter).setVelocity(targetSpeed);
        ((DcMotorEx) lowerShooter).setVelocity(targetSpeed);

        sleep(1000);

        while ( Math.abs(((DcMotorEx) upperShooter).getVelocity() - targetSpeed) > 10 || Math.abs(((DcMotorEx) lowerShooter).getVelocity() - targetSpeed) > 10) {
        }
        shooterPush.setPosition(shooterPushOnPosition);
        sleep(300);

        shooterPush.setPosition(shooterPushOffPosition);

        targetSpeed = 1085;

        ((DcMotorEx) upperShooter).setVelocity(targetSpeed);
        ((DcMotorEx) lowerShooter).setVelocity(targetSpeed);

        sleep (500);

        while ( Math.abs(((DcMotorEx) upperShooter).getVelocity() - targetSpeed) > 10 || Math.abs(((DcMotorEx) lowerShooter).getVelocity() - targetSpeed) > 10) {
        }

        shooterPush.setPosition(shooterPushOnPosition);

        sleep(300);

        ((DcMotorEx) upperShooter).setVelocity(0);
        ((DcMotorEx) lowerShooter).setVelocity(0);

        sleep(1000);

        shooterPush.setPosition(shooterPushOffPosition);

        intakeMotor.setPower(0.9);

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
        sleep(300);
        ((DcMotorEx) upperShooter).setVelocity(0);
        ((DcMotorEx) lowerShooter).setVelocity(0);
    }

    private void shootPins(int redBlueMultiplier){
        movement.pathFindToRestrictedDirectionNoStop(-5,44*redBlueMultiplier,-37*redBlueMultiplier,0.5,0.4,2,2);
        movement.runToPositionAndStop(-5,44*redBlueMultiplier,-38*redBlueMultiplier,0.4,1.7,1,0.5);

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

        movement.runToPositionAndStop(-5,44*redBlueMultiplier,-33*redBlueMultiplier,0.4,1.7,1,0.5);

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

        movement.runToPositionAndStop(-5,44*redBlueMultiplier,-31*redBlueMultiplier,0.4,1.7,1,0.5);

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
