package org.firstinspires.ftc.teamcode.autonomousAndTeleops.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.vision.Vuforia;
import org.firstinspires.ftc.teamcode.robot.vision.VuforiaFeedback;

@TeleOp(name = "VuforiaTest", group = "TeleOp")
@Disabled
// Replace SkeletonOpMode with name of OpMode


public class VuforiaTest extends LinearOpMode {


    @Override
    public void runOpMode() {

        //Initialization
        Vuforia myVuforia = new Vuforia(hardwareMap);
        //Declare local variables
        //Initialization code: example: map hardware

        //Wait for game to start
        waitForStart();

        //Sequential instruction for Autonomous
        //or while loop with update telemetry for driver control

        while (opModeIsActive()){
            VuforiaFeedback feedback = myVuforia.detectTarget();
            if (feedback.isSeenTarget()) {
                telemetry.addData("Name", feedback.getTargetName());
                telemetry.addData("X", feedback.getX());
                telemetry.addData("Y", feedback.getY());
                telemetry.addData("Angle", feedback.getAngle());
                telemetry.update();
            }

        }

//        myTensorFlow.stop();



    }
}

