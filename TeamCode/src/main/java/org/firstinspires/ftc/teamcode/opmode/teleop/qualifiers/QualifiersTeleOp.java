package org.firstinspires.ftc.teamcode.opmode.teleop.qualifiers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class QualifiersTeleOp extends LinearOpMode {

    public DcMotor [] motors;
    public double[] wheelPowers;
    public Servo [] servos;

    public double servoFlip;

    public float floatSlideKp;


    @Override
    public void runOpMode() throws InterruptedException{
        servoFlip = 0;

        // define motors //
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor rightFont = hardwareMap.dcMotor.get("rightFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor leftSlide = hardwareMap.dcMotor.get("leftSlide");
        DcMotor rightSlide = hardwareMap.dcMotor.get("rightSlide");

        // set motor directions //
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFont.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        Servo outake = hardwareMap.servo.get("outake");
        Servo leftOutakeFlip = hardwareMap.servo.get("leftOutakeFlip");
        Servo rightOutakeFlip = hardwareMap.servo.get("rightOutakeFlip");


        // list of all motors //
        motors = new DcMotor[]{leftFront, rightFont, leftBack, rightBack, intake, leftSlide, rightSlide};



        // reset motor and run with encoder //
        for (DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        servos = new Servo[]{outake, leftOutakeFlip, rightOutakeFlip};




        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){
            wheelPowers = getWheelPowers(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            for (int i = 0; i < 5; i++){
                motors[i].setPower(wheelPowers[i]);
            }

            if (gamepad1.options){
                imu.resetYaw();
            }

            if (gamepad1.right_trigger > gamepad1.left_trigger){
                intake.setPower(gamepad1.right_trigger);
            }
            else{
                intake.setPower(-gamepad1.left_trigger);
            }


        }
    }

    public double[] getWheelPowers(double x, double y, double rx, double botHeading){
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (rotY + rotX + rx) / denominator;
        double leftBackPower = (rotY - rotX + rx) / denominator;
        double rightFrontPower = (rotY - rotX - rx) / denominator;
        double rightBackPower = (rotY + rotX - rx) / denominator;
        double[] wheelPowers = new double[]{leftFrontPower, leftBackPower, rightFrontPower, rightBackPower};

        return (wheelPowers);
    }

    public void goToPositionPID(double Kp, double Ki, double Kd, double ticks, DcMotor motor, int currentPosition, int goal){
        double integralSum = 0;
        double lastError = 0;
        double error;
        double derivative;
        double power;

        ElapsedTime PIDTimer = new ElapsedTime();

        while (currentPosition != goal){
            currentPosition = motor.getCurrentPosition();
            error = goal - currentPosition;
            derivative = (error - lastError) / PIDTimer.seconds();
            integralSum = integralSum + (error * PIDTimer.seconds());

            power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
            motor.setPower(power);

            lastError = error;

            PIDTimer.reset();

        }
    }







}
