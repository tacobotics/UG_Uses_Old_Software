package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "TacoTele")

public class TacoTele extends Taco_Super_class {

    public VoltageSensor ControlHub_VoltageSensor;

    double Intake;
    double pos_arm;
    double pos_indexer;
    double Right_f;
    double Left_f;
    double Left_r;
    double Right_r;
    boolean do_we_smack_rings;

    @Override
    public void runOpMode() {

        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        Right_Front = hardwareMap.get(DcMotor.class, "Right_Front");
        Right_Rear = hardwareMap.get(DcMotor.class, "Right_Rear");
        Left_Front = hardwareMap.get(DcMotor.class, "Left_Front");
        Left_Rear = hardwareMap.get(DcMotor.class, "Left_Rear");
        arm = hardwareMap.get(DcMotor.class, "arm");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        wheels = hardwareMap.get(DcMotor.class, "wheels");
        indexer = hardwareMap.get(Servo.class, "indexer");
        pincher = hardwareMap.get(Servo.class, "pincher");
        ring_smacking_device_of_ring_smackingness = hardwareMap.get(Servo.class, "ring_smacking_device_of_ring_smackingness");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        // Reverse one of the drive motors.
        Intake = 1;
        pos_arm = 0;
        pos_indexer = 0.65;
        Left_r = 0;
        Left_f = 0;
        Right_f = 0;
        Right_r = 0;
        do_we_smack_rings = false;
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        wheels.setDirection(DcMotorSimple.Direction.REVERSE);
        //Right_Front.setDirection(DcMotorSimple.Direction.REVERSE);
        //Right_Rear.setDirection(DcMotorSimple.Direction.REVERSE);
        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Right_Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Left_Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Right_Rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        initialization(false);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
               /* Right_Front.setPower(Right_f);
                Right_Rear.setPower(Right_r);
                Left_Front.setPower(Left_f);
                Left_Rear.setPower(Left_r);*/
                indexer.setPosition(pos_indexer);
                pincher.setPosition(pos_arm);
                wheels.setPower(-Intake);
                intake2.setPower(-Intake*.75);
                drive();
                arm2();
                intake();
                indexer2();
                shooter2();
                arm_pinch();
                should_we_smack_da_rings();
                smack_a_da_rings();
            }
        }
    }

    private void indexer2() {
        if (!isStopRequested()) {
            if (0.5 < gamepad2.left_trigger) {
                pos_indexer = .3;
            } else {
                pos_indexer = 0.65;
            }
        }
    }

    private void should_we_smack_da_rings(){
        if(gamepad1.a){
            do_we_smack_rings = true;
        }
        else if(gamepad1.b){
            do_we_smack_rings = false;
        }
    }

    private void smack_a_da_rings(){
        if(do_we_smack_rings){
           ring_smacking_device_of_ring_smackingness.setPosition(.65);
        }
        else if (!do_we_smack_rings){
            ring_smacking_device_of_ring_smackingness.setPosition(1);
        }
    }

    private void drive() {
        if (!isStopRequested()) {
            if (gamepad1.right_bumper) {
                teledrive(gamepad1.left_stick_y/-2,gamepad1.left_stick_x/2,gamepad1.right_stick_x-gamepad1.left_stick_y/2);
            } else if (gamepad1.left_bumper) {
                teledrive(gamepad1.left_stick_y/-3,gamepad1.left_stick_x/3,gamepad1.right_stick_x-gamepad1.left_stick_y/3);
            } else {
                teledrive(-1*gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x-gamepad1.left_stick_y);

            }
        }
    }

    private void intake() {
        if (!isStopRequested()) {
            if (gamepad2.left_stick_y > 0.5) {
                Intake = 1;
            } else if (gamepad2.left_stick_y < -0.5) {
                Intake = -1;
            } else {
                Intake = 0;
            }
        }
    }

    private void shooter2() {
        if (!isStopRequested()) {
            shoot(gamepad2.right_bumper,gamepad2.right_trigger>.3, false);
        }
    }

    private void arm_pinch() {
        if (!isStopRequested()) {
            if (gamepad2.dpad_right) {
                pos_arm = 0;
            } else if (gamepad2.dpad_left) {
                pos_arm = 1;
            } else if (gamepad2.left_bumper) {
                pos_arm = 0.5;
            }
        }
    }

    private void arm2() {
        if (!isStopRequested()) {
            if (gamepad2.dpad_up) {
                arm.setPower(0.75);
            } else if (gamepad2.dpad_down) {
                arm.setPower(-0.75);
            } else {
                arm.setPower(0);
            }
        }
    }
}
