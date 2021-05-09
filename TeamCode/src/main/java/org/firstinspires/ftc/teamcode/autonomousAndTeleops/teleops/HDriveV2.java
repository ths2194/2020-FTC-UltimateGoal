package org.firstinspires.ftc.teamcode.autonomousAndTeleops.teleops;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.teamcode.opModes.MyOpMode;
import org.firstinspires.ftc.teamcode.robot.robotClasses.RobotMovement;

import java.util.StringTokenizer;

import static java.lang.Math.abs;
import static java.lang.Math.max;

@TeleOp(name = "HDriveV2", group = "TeleOp")

public class HDriveV2 extends MyOpMode {
    @Override
    public void runOpMode() {
        initializeWithOdometry();
        RobotMovement movement = new RobotMovement(FRM,FLM,BRM,BLM,globalPositionUpdate,this);

        double intakeMotorPower = 0.9;
        double targetShooterVelocity = 1465;

        double shootX = -5;
        double shootY = 32;
        double shootAngle = -5;

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
                        movement.setMotorPowers(FLPower+controller1.left_stick_x*.2,FRPower-controller1.left_stick_x*.2,BLPower-controller1.left_stick_x*.2,BRPower+controller1.left_stick_x*.2,.65);
                    } else if (controller1.leftBumper()) {
                        movement.setMotorPowers(FLPower,FRPower,BLPower,BRPower,0.35);
                    } else {
                        movement.setMotorPowers(FLPower,FRPower,BLPower,BRPower,0.55);
                    }
                } else {
                    double dist = Math.hypot(shootX-globalPositionUpdate.returnXCoordinate(), shootY-globalPositionUpdate.returnYCoordinate());
                    double minPow = 0.2; double lowDistBound = 1; double highDistBound = 3;
                    double power = (.65-minPow)/(highDistBound-lowDistBound)*(dist-lowDistBound)+minPow;
                    power = Range.clip(power,0.2,.65);
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

                if (controller1.dpadUpOnce()) {
                    targetShooterVelocity += 10;
                }
                else if (controller1.dpadDownOnce()) {
                    targetShooterVelocity -= 10;
                }

                if (controller1.XOnce()){
                    intakeMult = Math.abs(intakeMult - 1); //toggles between 1 and 0
                }

                if (controller1.YOnce()) {
                    runningToPosition = !runningToPosition;
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
                    if (getRuntime() - timeStamp > 1.5) autoSequenceStep++;
                }
                if (autoSequenceStep == 3) {
                    if ((Math.abs(((DcMotorEx) upperShooter).getVelocity() - targetShooterVelocity) < 10 && Math.abs(((DcMotorEx) lowerShooter).getVelocity() - targetShooterVelocity) < 10) ) {
                        shooterPush.setPosition(shooterPushOnPosition);
                        timeStamp = getRuntime();
                        autoSequenceStep++;
                    }
                }

                if (autoSequenceStep == 4) {
                    if (getRuntime() - timeStamp > 0.3) {
                        shooterPush.setPosition(shooterPushOffPosition);
                        shootersOn = false;
                        autoSequenceStep++;
                    }
                }

                if (controller1.AOnce() || controller2.AOnce()) {
                    runningToPosition = false;
                    intakeMult = 0;
                    shootersOn = false;
                    autoSequenceStep = 0;
                    gripper.setPosition(0.2);
                    shooterPush.setPosition(shooterPushOffPosition);
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

                intakeMotor.setPower(intakeMotorPower * intakeMult
                        + 0.5*(controller1.right_trigger - controller1.left_trigger) );


                if (shootersOn) {
                    ((DcMotorEx) upperShooter).setVelocity(targetShooterVelocity);
                    ((DcMotorEx) lowerShooter).setVelocity(targetShooterVelocity);
                }
                else {
                    ((DcMotorEx) upperShooter).setVelocity(0);
                    ((DcMotorEx) lowerShooter).setVelocity(0);
                }


                if (controller2.rightBumperOnce()) {
                    if (shooterPush.getPosition() == shooterPushOffPosition) shooterPush.setPosition(shooterPushOnPosition);
                    else shooterPush.setPosition(shooterPushOffPosition);
                }


                if (controller2.dpadUpOnce()) {
                    if (controller2.leftBumper()) {
                        shootX += 1;
                    }
                    else {
                        targetShooterVelocity += 10;
                    }
                }
                else if (controller2.dpadDownOnce()) {
                    if (controller2.leftBumper()) {
                        shootX -= 1;
                    }
                    else {
                        targetShooterVelocity -= 10;
                    }
                }

                if (controller2.dpadLeftOnce()) {
                    if (controller2.leftBumper()) {
                        shootY += 1;
                    }
                    else {
                        shootAngle += 1;
                    }
                }
                else if (controller2.dpadRightOnce()) {
                    if (controller2.leftBumper()) {
                        shootY -= 1;
                    }
                    else {
                        shootAngle -= 1;
                    }
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
            globalPositionUpdate.setPosition(10, 29, 0);
        } else {
            StringTokenizer tok = new StringTokenizer(endPositionValues);
            double xPos = Double.parseDouble(tok.nextToken());
            double yPos = Double.parseDouble(tok.nextToken());
            double robotAngle = Double.parseDouble(tok.nextToken());
            globalPositionUpdate.setPosition(xPos,yPos,robotAngle);
        }
    }

}

