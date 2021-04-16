package org.firstinspires.ftc.teamcode.autonomousAndTeleops.teleops;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.opModes.MyOpMode;
import org.firstinspires.ftc.teamcode.robot.robotClasses.RobotMovement;

import static java.lang.Math.abs;
import static java.lang.Math.max;

@TeleOp(name = "MecanumWheelDrive", group = "TeleOp")

public class MecanumWheelDrive extends MyOpMode {

    @Override
    public void runOpMode() {
        initializeWithOdometry();
        RobotMovement movement = new RobotMovement(FRM,FLM,BRM,BLM,telemetry,globalPositionUpdate);

        double directionMultiplier = 1;
        double intakeMotorPower = 0.7;
        double targetShooterVelocity = 1055;

        double shootX = -5;
        double shootY = 32;
        double shootAngle = -5;

        boolean readyToShoot = false;
        boolean runningToPosition = false;

        int intakeMult = 0;
        boolean shootersOn = false;

        gripper.setPosition(0.2);

        waitForStart();
        globalPositionUpdate.setPosition(10, 29, 0);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                controller1.update();
                controller2.update();

                double FRPower, BRPower, FLPower, BLPower;

                FRPower = -controller1.left_stick_y - controller1.left_stick_x - 0.5*directionMultiplier*controller1.right_stick_x;
                BRPower = -controller1.left_stick_y + controller1.left_stick_x - 0.5*directionMultiplier*controller1.right_stick_x;
                FLPower = -controller1.left_stick_y + controller1.left_stick_x + 0.5*directionMultiplier*controller1.right_stick_x;
                BLPower = -controller1.left_stick_y - controller1.left_stick_x + 0.5*directionMultiplier*controller1.right_stick_x;

                double maxPower = max(max(max(abs(FRPower), abs(BRPower)), abs(FLPower)), abs(BLPower));
                FRPower /= maxPower;
                BRPower /= maxPower;
                FLPower /= maxPower;
                BLPower /= maxPower;

                // set power
                if (!runningToPosition) {
                    if (controller1.left_trigger > 0) {
                        setMotorPowers(FLPower,FRPower,BLPower,BRPower,directionMultiplier);
                    } else if (controller1.leftBumper()) {
                        setMotorPowers(FLPower,FRPower,BLPower,BRPower,directionMultiplier*0.2);
                    } else {
                        setMotorPowers(FLPower,FRPower,BLPower,BRPower,directionMultiplier*0.5);
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

                if (controller1.AOnce() || controller2.AOnce()){
                    if (intakeMult != 1) { intakeMult = 1; } else { intakeMult = 0; }
                }
                else if (controller1.YOnce() || controller2.YOnce()){
                    if (intakeMult != -1) { intakeMult = -1; } else { intakeMult = 0; }
                }

                if (controller1.dpadUpOnce()){
                    intakeMotorPower = Math.min(intakeMotorPower+0.1,1);
                }
                else if (controller1.dpadDownOnce()){
                    intakeMotorPower = Math.max(intakeMotorPower-0.1,-1);
                }

                if (controller1.leftTriggerOnce() && controller1.rightTriggerOnce()){
                    directionMultiplier *= -1;
                }

                if (controller2.BOnce()) {
                    shootersOn = !shootersOn;
                }

                if (controller1.dpadRightOnce()) {
                    if (controller1.left_trigger > 0){ targetShooterVelocity += 5; } else { targetShooterVelocity += 10; }
                }
                else if (controller1.dpadLeftOnce()) {
                    if (controller1.left_trigger > 0){ targetShooterVelocity -= 5; } else { targetShooterVelocity -= 10; }
                }

                if (controller1.XOnce()) {
                    if (gripper.getPosition() != 1){
                        gripper.setPosition(1);
                    }
                    else {
                        gripper.setPosition(0);
                    }
                }

                if (controller2.rightBumperOnce()) {
                    readyToShoot = !readyToShoot;
                }

                if (controller2.XOnce()) {
                    readyToShoot = false;
                }

                if (Math.hypot(controller1.left_stick_x,controller1.left_stick_y) > 0.05 || Math.hypot(controller1.right_stick_x,controller1.right_stick_y) > 0.05){
                    runningToPosition = false;
                }

                if (controller1.rightStickButtonOnce()) {
                    runningToPosition = !runningToPosition;
                }

                if (controller1.leftStickButton()) {
                    shootX = globalPositionUpdate.returnXCoordinate();
                    shootY = globalPositionUpdate.returnYCoordinate();
                    shootAngle = globalPositionUpdate.returnOrientation();
                }

                if (controller2.dpadUpOnce()) {
                    if (controller2.left_trigger > 0){
                        targetShooterVelocity += 5;
                    }
                    else if (controller2.leftBumper()) {
                        shootX += 1;
                    }
                    else {
                        targetShooterVelocity += 1;
                    }
                }
                else if (controller2.dpadDownOnce()) {
                    if (controller2.left_trigger > 0){
                        targetShooterVelocity -= 5;
                    }
                    else if (controller2.leftBumper()) {
                        shootX -= 1;
                    }
                    else {
                        targetShooterVelocity -= 1;
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


                intakeMotor.setPower(intakeMotorPower * intakeMult);
                if (shootersOn) {
                    ((DcMotorEx) upperShooter).setVelocity(targetShooterVelocity);
                    ((DcMotorEx) lowerShooter).setVelocity(targetShooterVelocity);
                }
                else {
                    ((DcMotorEx) upperShooter).setVelocity(0);
                    ((DcMotorEx) lowerShooter).setVelocity(0);
                }
                if (readyToShoot) {
                    if ( Math.abs(((DcMotorEx) upperShooter).getVelocity() - targetShooterVelocity) < 20 && Math.abs(((DcMotorEx) lowerShooter).getVelocity() - targetShooterVelocity) < 20) {
                        shooterPush.setPosition(shooterPushOnPosition);
                    }
//                    shooterPush.setPosition(shooterPushOnPosition);
                }
                else {
                    shooterPush.setPosition(shooterPushOffPosition);
                }


                if (runningToPosition) telemetry.addData("Running To Position","In Progress");
                if (readyToShoot) telemetry.addData("Ready To Shoot", "Waiting For Motors");
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


}

