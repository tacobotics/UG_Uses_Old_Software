package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

import java.util.List;

@Autonomous(name = "java_auto_go")

public class java_auto_go extends Taco_Super_class {

    double Shooter_Speed;
    double Rounds;
    double Zone; //stuff for vision
    double index; //stuff for vision

    @Override
    public void runOpMode() {
        List<Recognition> recognitions; //stuff for vision
        Right_Front = hardwareMap.get(DcMotor.class, "Right_Front");
        Right_Rear = hardwareMap.get(DcMotor.class, "Right_Rear");
        Left_Front = hardwareMap.get(DcMotor.class, "Left_Front");
        Left_Rear = hardwareMap.get(DcMotor.class, "Left_Rear");
        arm = hardwareMap.get(DcMotor.class, "arm");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        wheels = hardwareMap.get(DcMotor.class, "wheels");
        indexer = hardwareMap.get(Servo.class, "indexer");
        pincher = hardwareMap.get(Servo.class, "pincher");

        initialization(true);

        telemetry.addData(">", "Wait To Start!!!!!!!!!!!!!!!!!!");
        telemetry.update();
        Shooter_Speed = 0.65;
        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        wheels.setDirection(DcMotorSimple.Direction.REVERSE);
        androidSoundPool = new AndroidSoundPool(); //stuff for sound
        vuforiaUltimateGoal = new VuforiaCurrentGame(); //stuff for vision
        tfodUltimateGoal = new TfodCurrentGame(); //stuff for vision
        Zone = 1; //Stuff for vision

        /* stuff for vision below */
        vuforiaUltimateGoal.initialize(
                "", // vuforiaLicenseKey
                hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
                "", // webcamCalibrationFilename
                false, // useExtendedTracking
                true, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                0, // xAngle
                0, // yAngle
                0, // zAngle
                true); // useCompetitionFieldTargetLocations
        // Set min confidence threshold to 0.7
        tfodUltimateGoal.initialize(vuforiaUltimateGoal, 0.7F, true, true);
        // Initialize TFOD before waitForStart.
        // Init TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfodUltimateGoal.activate();
        // Enable following block to zoom in on target.
        tfodUltimateGoal.setZoom(2.5, 1.7777777777777777777777777777777);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        androidSoundPool.play("RawRes:ss_roger_roger");//sound stuff

        waitForStart();
        if(opModeIsActive()){
            if (opModeIsActive()) {
                // Get a list of recognitions from TFOD.
                recognitions = tfodUltimateGoal.getRecognitions();
                while (Rounds < 16) {
                    // If list is empty, inform the user. Otherwise, go
                    // through list and display info for each recognition.
                    if (recognitions.size() == 0) {
                        Zone = 1;
                        telemetry.addData("Zone", Zone);
                        Rounds += 1;
                        telemetry.addData("Zone", Zone);
                        telemetry.addData("Rounds", Rounds);
                        telemetry.update();
                    } else {
                        index = 0;
                        // Iterate through list and call a function to
                        // display info for each recognized object.
                        for (Recognition recognition_item : recognitions) {
                            recognition = recognition_item;
                            // Display info.
                            displayInfo(index);
                            // Increment index.
                            index = index + 1;
                            Object2();
                            telemetry.addData("Zone", Zone);
                            telemetry.addData("Zone", Zone);
                            telemetry.addData("Rounds", Rounds);
                            telemetry.update();
                        }
                    }
                }
            }
            if (Zone == 1) {
                Zone_A();
            } else if (Zone == 2) {
                Zone_B();
            } else if (Zone == 3) {
                Zone_C();
            }
        }

        }

    public void Zone_A() {
        driveForwardEncoders(1000,1);
        sleep(500);
        driveright(1050,.7);
        driveForwardEncoders(1250,.6);
        sleep(700);
        Shooter_Speed = 0.67;
        sleep(10);

        shoot(false,false, true);
        sleep(500);
        shoot(false,false, true);
        indexer.setPosition(0);
        shoot(false,false, true);
        sleep(750);
        shoot(false,false, true);
        indexer.setPosition(0.65);
        shoot(false,false, true);
        sleep(750);
        shoot(false,false, true);
        indexer.setPosition(0);
        shoot(false,false, true);
        sleep(750);
        shoot(false,false, true);
        indexer.setPosition(0.65);
        shoot(false,false, false);
        shoot(false,false, true);
        sleep(500);
        shoot(false,false, true);
        indexer.setPosition(0);
        shoot(false,false, true);
        sleep(750);
        shoot(false,false, true);
        indexer.setPosition(0.65);
        shoot(false,false, true);
        sleep(750);
        shoot(false,false, true);
        indexer.setPosition(0);
        shoot(false,false, true);
        sleep(750);
        shoot(false,false, true);
        indexer.setPosition(0.65);
        shoot(false,false, false);

        driveleft(300,.5);
        driveForwardEncoders(500,.7);
        //drop it
        pincher.setPosition(0);
        driveBackwardEncoders(2900,5);
        sleep(1000);
        pincher.setPosition(.99);
        driveForwardEncoders(2900,.5);
        pincher.setPosition(0);
        driveBackwardEncoders(1000,1);
        sleep(30000);
    }


    public void Zone_B() {
        driveForwardEncoders(1000,.8);
        sleep(500);
        driveright(1050,.7);
        driveForwardEncoders(1250,.6);
        sleep(700);
        Shooter_Speed = 0.67;
        sleep(10);

        shoot(false,false, true);
        sleep(500);
        shoot(false,false, true);
        indexer.setPosition(0);
        shoot(false,false, true);
        sleep(750);
        shoot(false,false, true);
        indexer.setPosition(0.65);
        shoot(false,false, true);
        sleep(750);
        shoot(false,false, true);
        indexer.setPosition(0);
        shoot(false,false, true);
        sleep(750);
        shoot(false,false, true);
        indexer.setPosition(0.65);
        shoot(false,false, false);
        shoot(false,false, true);
        sleep(500);
        shoot(false,false, true);
        indexer.setPosition(0);
        shoot(false,false, true);
        sleep(750);
        shoot(false,false, true);
        indexer.setPosition(0.65);
        shoot(false,false, true);
        sleep(750);
        shoot(false,false, true);
        indexer.setPosition(0);
        shoot(false,false, true);
        sleep(750);
        shoot(false,false, true);
        indexer.setPosition(0.65);
        shoot(false,false, false);

        driveleft(300,.5);

        driveright(750,1);

        //driveleft(1500,.8);
        driveForwardEncoders(1500,.8);
        driveBackwardEncoders(1500,.8);
        driveleft(750,.4);
        driveForwardEncoders(2900,5);
        driveBackwardEncoders(2900,.5);
        //wobbleClawL.setposition(ungrab);
        //wobbleClawR.setposition(ungrab);
        driveBackwardEncoders(1000,.7);
    }


    public void Zone_C() {
        driveForwardEncoders(1000,.8);
        //wobbleClawL.setposition(grab);
        //wobbleClawR.setposition(grab);
        sleep(500);
        driveright(1050,.7);
        driveForwardEncoders(1250,.6);
        sleep(700);
        Shooter_Speed = 0.67;
        sleep(10);

        shoot(false,false, true);
        sleep(500);
        shoot(false,false, true);
        indexer.setPosition(0);
        shoot(false,false, true);
        sleep(750);
        shoot(false,false, true);
        indexer.setPosition(0.65);
        shoot(false,false, true);
        sleep(750);
        shoot(false,false, true);
        indexer.setPosition(0);
        shoot(false,false, true);
        sleep(750);
        shoot(false,false, true);
        indexer.setPosition(0.65);
        shoot(false,false, false);
        shoot(false,false, true);
        sleep(500);
        shoot(false,false, true);
        indexer.setPosition(0);
        shoot(false,false, true);
        sleep(750);
        shoot(false,false, true);
        indexer.setPosition(0.65);
        shoot(false,false, true);
        sleep(750);
        shoot(false,false, true);
        indexer.setPosition(0);
        shoot(false,false, true);
        sleep(750);
        shoot(false,false, true);
        indexer.setPosition(0.65);
        shoot(false,false, false);


        driveleft(300,.5);
        while (nextstep(0,0,2000)) {
            teledrive(foregoal(0), rightgoal(0),turngoal(2000));
        }
        teledrive(0,0,0);
        driveright(3000,.7);
        driveleft(2900,5);
        //wobbleClawL.setposition(grab);
        //wobbleClawR.setposition(grab);
        driveright(2900,.5);
        //wobbleClawL.setposition(ungrab);
        //wobbleClawR.setposition(ungrab);
        driveBackwardEncoders(1000,1);
        sleep(30000);
    }
}
