package org.firstinspires.ftc.teamcode.autonomousAndTeleops.teleops;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.teamcode.opModes.MyOpMode;
import org.firstinspires.ftc.teamcode.robot.robotClasses.RobotMovement;

import java.util.StringTokenizer;

import static java.lang.Math.abs;
import static java.lang.Math.max;

@TeleOp(name = "BenDrive", group = "TeleOp")
@Disabled

public class BenDrive extends MyOpMode {
    int redBlueMultiplier;
    @Override
    public void runOpMode() {
        initializeWithOdometry();
        RobotMovement movement = new RobotMovement(FRM,FLM,BRM,BLM,globalPositionUpdate,this);
        redBlueMultiplier = onBlueSide? 1 : -1;

        double intakeMotorPower = 0.9;
        double targetShooterVelocity = 1055;

        double shootX = -5;
        double shootY = 32*redBlueMultiplier;
        double shootAngle = -5*redBlueMultiplier;

        boolean runningToPosition = false;

        int intakeMult = 0;
        boolean shootersOn = false;

        int autoSequenceStep = 0;
        double timeStamp = 0;

        gripper.setPosition(0.2);
        shooterPush.setPosition(shooterPushOffPosition);

        waitForStart();
        initializeGlobalPosition();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                controller1.update();
                controller2.update();

                double FRPower, BRPower, FLPower, BLPower;

                FRPower = -controller1.left_stick_y - controller1.left_stick_x - 0.5*controller1.right_stick_x;
                BRPower = -controller1.left_stick_y + controller1.left_stick_x - 0.5*controller1.right_stick_x;
                FLPower = -controller1.left_stick_y + controller1.left_stick_x + 0.5*controller1.right_stick_x;
                BLPower = -controller1.left_stick_y - controller1.left_stick_x + 0.5*controller1.right_stick_x;

                double maxPower = max(max(max(abs(FRPower), abs(BRPower)), abs(FLPower)), abs(BLPower));
                FRPower /= maxPower;
                BRPower /= maxPower;
                FLPower /= maxPower;
                BLPower /= maxPower;

                // set power
                if (!runningToPosition) {
                    if (controller1.rightBumper()) {
                        movement.setMotorPowers(FLPower,FRPower,BLPower,BRPower);
                    } else if (controller1.leftBumper()) {
                        movement.setMotorPowers(FLPower,FRPower,BLPower,BRPower,0.5);
                    } else {
                        movement.setMotorPowers(FLPower,FRPower,BLPower,BRPower,0.65);
                    }
                } else {
                    double dist = Math.hypot(shootX-globalPositionUpdate.returnXCoordinate(), shootY-globalPositionUpdate.returnYCoordinate());
                    double minPow = 0.2; double lowDistBound = 1; double highDistBound = 3;
                    double power = (.7-minPow)/(highDistBound-lowDistBound)*(dist-lowDistBound)+minPow;
                    power = Range.clip(power,0.2,.7);
                    if ( Math.abs(shootX-globalPositionUpdate.returnXCoordinate()) > 2 || Math.abs(shootY-globalPositionUpdate.returnYCoordinate()) > 2
                            || (Math.abs(shootAngle - globalPositionUpdate.returnOrientation()) + 2)%360 - 2 > 2) {
                        movement.setTargetPowers(shootX, shootY, shootAngle, power, 0.4);
                    }
                    else {
                        movement.stop();
                    }
                }

                if (Math.hypot(controller1.left_stick_x,controller1.left_stick_y) > 0.05 || Math.hypot(controller1.right_stick_x,controller1.right_stick_y) > 0.05){
                    runningToPosition = false;
                }

                if (controller1.dpadRightOnce()){
                    intakeMotorPower = Math.min(intakeMotorPower+0.1,1);
                }
                else if (controller1.dpadLeftOnce()){
                    intakeMotorPower = Math.max(intakeMotorPower-0.1,-1);
                }

                if (controller1.dpadUpOnce() || controller2.rightBumperOnce()) {
                    targetShooterVelocity += 1;
                }
                else if (controller1.dpadDownOnce() || controller2.leftBumperOnce()) {
                    targetShooterVelocity -= 1;
                }

                if (controller1.AOnce()) {
                    runningToPosition = !runningToPosition;
                }

                if (controller1.XOnce()){
                    intakeMult = Math.abs(intakeMult - 1); //toggles between 1 and 0
                }

                if (controller1.YOnce()) {
                    shootersOn = !shootersOn;
                    timeStamp = getRuntime();
                }

                if (controller1.BOnce()) {
                    autoSequenceStep = 1;
                }

                if (autoSequenceStep == 1) {
                    if (!shootersOn) {
                        shootersOn = true;
                        timeStamp = getRuntime();
                    }
                    autoSequenceStep++;
                }
                if (autoSequenceStep == 2) {
                    if (getRuntime() - timeStamp > 1) autoSequenceStep++;
                }
                if (autoSequenceStep == 3) {
                    if ((Math.abs(((DcMotorEx) upperShooter).getVelocity() - targetShooterVelocity) < 20 && Math.abs(((DcMotorEx) lowerShooter).getVelocity() - targetShooterVelocity) < 20) ) {
                        shooterPush.setPosition(shooterPushOnPosition);
                        timeStamp = getRuntime();
                        autoSequenceStep++;
                    }
                }

                if (autoSequenceStep == 4) {
                    if (getRuntime() - timeStamp > 0.5) {
                        shooterPush.setPosition(shooterPushOffPosition);
                        shootersOn = false;
                        intakeMult = 1;
                        autoSequenceStep++;
                    }
                }

                if (controller1.backOnce()) {
                    if (gripper.getPosition() != 1){
                        gripper.setPosition(1);
                    }
                    else {
                        gripper.setPosition(0);
                    }
                }

                if (controller1.start()) {
                    shootX = globalPositionUpdate.returnXCoordinate();
                    shootY = globalPositionUpdate.returnYCoordinate();
                    shootAngle = globalPositionUpdate.returnOrientation();
                }

                if (controller2.dpadUpOnce()) {
                    shootX++;
                }
                else if (controller2.dpadDownOnce()) {
                    shootX--;
                }
                else if (controller2.dpadLeftOnce()) {
                    shootAngle++;
                }
                else if (controller2.dpadRightOnce()) {
                    shootAngle--;
                }


                if (controller2.A()) {
                    shootersOn = false;
                }
                else if (controller2.B()) {
                    shootersOn  = true;
                }
                if (controller2.X()) {
                    intakeMult = 0;
                }
                else if (controller2.Y()) {
                    intakeMult = 1;
                }

                if (controller2.back()) {
                    shooterPush.setPosition(shooterPushOffPosition);
                }
                else if (controller2.start()) {
                    shooterPush.setPosition(shooterPushOnPosition);
                }


                intakeMotor.setPower(intakeMotorPower * intakeMult
                        + (controller2.right_trigger + controller1.right_trigger - controller2.left_trigger - controller1.left_trigger) );


                if (shootersOn) {
                    ((DcMotorEx) upperShooter).setVelocity(targetShooterVelocity);
                    ((DcMotorEx) lowerShooter).setVelocity(targetShooterVelocity);
                }
                else {
                    ((DcMotorEx) upperShooter).setVelocity(0);
                    ((DcMotorEx) lowerShooter).setVelocity(0);
                }


                if (runningToPosition) telemetry.addData("Running To Position","In Progress");
                telemetry.addData("IntakeMotorPower",intakeMotor.getPower());
                telemetry.addData("Target Shooter Velocity", targetShooterVelocity);
                telemetry.addData("Actual Shooter Velocity", ((DcMotorEx) upperShooter).getVelocity());

                telemetry.addData("XPos", globalPositionUpdate.returnXCoordinate());
                telemetry.addData("YPos", globalPositionUpdate.returnYCoordinate());
                telemetry.addData("Angle", globalPositionUpdate.returnOrientation());


                telemetry.update();

            }
        }

    }

    void initializeGlobalPosition () {
        String endPositionValues = ReadWriteFile.readFile(endPosition).trim();
        if (endPositionValues.equals("")) {
            globalPositionUpdate.setPosition(10, 29*redBlueMultiplier, 0*redBlueMultiplier);
        } else {
            StringTokenizer tok = new StringTokenizer(endPositionValues);
            double xPos = Double.parseDouble(tok.nextToken());
            double yPos = Double.parseDouble(tok.nextToken());
            double robotAngle = Double.parseDouble(tok.nextToken());
            globalPositionUpdate.setPosition(xPos,yPos,robotAngle);
        }
    }

}

