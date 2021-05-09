package org.firstinspires.ftc.teamcode.autonomousAndTeleops.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opModes.MyOpMode;
import org.firstinspires.ftc.teamcode.robot.odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.robot.robotClasses.RobotMovement;
import org.firstinspires.ftc.teamcode.robot.vision.TensorFlow;


@Autonomous(name = "AutonomousTry2")
public class AutonomousTry2 extends MyOpMode {

    boolean onLeft = true;
    TensorFlow myTensorFlow;
    RobotMovement movement;

    void initialize() {
        initializeWithOdometry();
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setTargetPosition(576);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        gripper.setPosition(0); //change to 0
        myTensorFlow = new TensorFlow(hardwareMap);
        movement = new RobotMovement(FRM,FLM,BRM,BLM,globalPositionUpdate,this);
        shooterPush.setPosition(shooterPushOffPosition);
    }

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

        while (opModeIsActive() && numOfRings == 0 && getRuntime()-startTimeForDetection < 1.5) {
            numOfRings = myTensorFlow.getNumberOfRings();
        }
        return numOfRings;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStartMenu();
        int redBlueMultiplier = onBlueSide? 1 : -1;
        double startX, startY, startAngle;

//        BEGIN OP MODE HERE:
        int numOfRings;
        if (onLeft) {
            globalPositionUpdate.setPosition(-62,12,0);
            movement.runToPositionAndStop(-54,13,-20,0.5,0.5,3,3);
            numOfRings = detectRings();
            movement.runToPosition(-54,14,0,0.6,0.6,3,3);
            movement.runToPosition(-5,14,0,0.5,0.4,3,3);
            movement.runToPositionAndStop(-5,-3,-8,0.5,0.9,1,3);
        }
        else {
            globalPositionUpdate.setPosition(-62,-12,0);
            movement.runToPositionAndStop(-54,-13,25,0.5,0.5,3,3);
            numOfRings = detectRings();
            movement.runToPosition(-54,-14,0,0.6,0.6,3,3);
            movement.runToPosition(-5,-14,0,0.5,0.4,3,3);
            movement.runToPositionAndStop(-5,0,-8,0.5,0.9,3,3);
        }

        telemetry.addData("HERE 1","OK");
        telemetry.update();

        if (opModeIsActive()) shoot(1435,200);
        if (opModeIsActive()) shootAndTurnOffMotors(1445,400);
        telemetry.addData("HERE 2","OK");
        telemetry.update();

        if (opModeIsActive()) intakeMotor.setPower(0.9);
        telemetry.addData("HERE 2.5","OK");
        telemetry.update();

        while (opModeIsActive() && intakeMotor.getCurrentPosition() != intakeMotor.getTargetPosition()){
        }

        telemetry.addData("HERE 3","OK");
        telemetry.update();

        if (opModeIsActive()) sleep(200);

        telemetry.addData("HERE 3.5","OK");
        telemetry.update();

        if (opModeIsActive()) intakeMotor.setPower(0);

        telemetry.addData("HERE 4","OK");
        telemetry.update();

        if (opModeIsActive()) shootAndTurnOffMotors(1445,400);

        telemetry.addData("HERE 5","OK");
        telemetry.update();

        if (numOfRings == 0) {
            movement.runToPositionAndStop(-6, 22*redBlueMultiplier, 0*redBlueMultiplier, 0.7, 0.4, 2, 10);
        }
        else if (numOfRings == 4) {
            movement.runToPositionAndStop(42, 22*redBlueMultiplier, 0*redBlueMultiplier, 0.7, 0.4, 2, 10);
        }
        else if (numOfRings == 1){
            movement.runToPosition(16, 14*redBlueMultiplier, -10*redBlueMultiplier, 0.7, 0.4, 2, 10);
            movement.runToPositionAndStop(24, 9*redBlueMultiplier, -45*redBlueMultiplier, 0.7, 0.6, 2, 10);
        }

        if (opModeIsActive()) sleep(100);

        gripper.setPosition(0.44);

        if (numOfRings == 0) {
            movement.runToPosition(-12, 22*redBlueMultiplier, 0*redBlueMultiplier, 0.7, 0.4, 2, 10);
            movement.runToPosition(-12, 0*redBlueMultiplier, 0*redBlueMultiplier, 0.9, 0.4, 2, 10);
        }

        movement.runToPositionAndStop(10, 0*redBlueMultiplier, 0*redBlueMultiplier, 0.7, 0.4, 2, 10);

        ReadWriteFile.writeFile(endPosition, globalPositionUpdate.returnXCoordinate() + " " + globalPositionUpdate.returnYCoordinate() + " " + globalPositionUpdate.returnOrientation());

        while (opModeIsActive()) {
            //Display Global (x, y, theta) coordinates
            displayTelemetry(telemetry,globalPositionUpdate,Integer.toString(numOfRings));
        }
        //Stop the thread
        globalPositionUpdate.stop();

    }

    void shoot(double targetSpeed, long retractTime) {
        setShootMotors(targetSpeed);

        if (!opModeIsActive()) return;

        sleep(1000);

        while (opModeIsActive() && Math.abs(((DcMotorEx) upperShooter).getVelocity() - targetSpeed) > 10 || Math.abs(((DcMotorEx) lowerShooter).getVelocity() - targetSpeed) > 10) {
        }

        shooterPush.setPosition(shooterPushOnPosition);

        if (!opModeIsActive()) return;
        sleep(retractTime);

        shooterPush.setPosition(shooterPushOffPosition);
    }

    void setShootMotors (double targetSpeed) {
        if (!opModeIsActive()) return;
        ((DcMotorEx) upperShooter).setVelocity(targetSpeed);
        sleep(200);
        ((DcMotorEx) lowerShooter).setVelocity(targetSpeed);
    }

    void shootAndTurnOffMotors (double targetSpeed, long retractTime) {
        shoot(targetSpeed, retractTime);
        setShootMotors(0);
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
