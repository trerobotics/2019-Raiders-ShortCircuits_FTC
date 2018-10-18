package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import edu.spa.ftclib.internal.activator.MotorActivator;
import edu.spa.ftclib.internal.controller.PIDController;
import edu.spa.ftclib.internal.drivetrain.TankDrivetrain;
import edu.spa.ftclib.internal.state.ToggleBoolean;

public class Robot
{
    /* local Setup Stuff */
    private HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Drive Motors
     * Not every motor is being used right now as
     * this is just thrown together for Makerfest.
    */
    public DcMotor leftFrontDrive   = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive  = null;
    public DcMotor rightBackDrive = null;


    /*
     * Motors for arm and intake
     */
    public MotorActivator intake = null;
    public DcMotor armAngle = null;
    public DcMotor armExtension = null;
    public DcMotor armHeight = null;

    /*
     * Arrays for encoder values.
     * Array[0] is smaller, Array[1] is the larger.
     */
    private int[] armHeightPos = new int[]{0, 2000};
    private int[] armExtensionPos = new int[]{0, 800};

    /*
     * Power values for Motor to change height and extend arm.
     */

    private int armHeightPower = 1;
    private int armExtensionPower = 1;

    /*
     * Using the HOMAR library drivetrain for some testing.
     * might not work for four wheel drive and will probably have to
     * write new stuff.
     */
    private TankDrivetrain drivetrain;

    /*
     * dismiss for now.
     */
    private DcMotor[] driveTrainLeftMotors;
    private DcMotor[] driveTrainRightMotors;

    /*
     * Imu variables
     */
    private BNO055IMU imu = null;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    /*
     * PID controller for chassis rotation.
     */
    PIDController pidRotate;

    /* Constructor */
    public Robot(HardwareMap hwMap){
        this.hwMap = hwMap;
    }

    public void init()
    {
        //leftFrontDrive = hwMap.get(DcMotor.class, "left_front");
        leftBackDrive = hwMap.get(DcMotor.class, "left_back");
        //
        // rightFrontDrive = hwMap.get(DcMotor.class, "right_front");
        rightBackDrive = hwMap.get(DcMotor.class, "right_back");
        driveTrainLeftMotors = new DcMotor[]{leftBackDrive, leftFrontDrive};
        driveTrainRightMotors = new DcMotor[]{rightBackDrive, rightFrontDrive};


        intake = hwMap.get(MotorActivator.class, "intake");
        armAngle = hwMap.get(DcMotor.class, "arm_angle");
        armExtension = hwMap.get(DcMotor.class, "arm_extension");
        armHeight = hwMap.get(DcMotor.class, "arm_height");


        pidRotate = new PIDController(.0005,0,0);

        drivetrain = new TankDrivetrain(leftBackDrive, rightBackDrive);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

    }

    public void stop()
    {
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    /*
     * only take the largest value and return that.
     * used for drivetrain simplification
     */
    private double absMax(double a, double b) { //Returns the argument whose absolute value is greater (similar to Math.max() but compares absolute values)
        return (Math.abs(a) > Math.abs(b)) ? a : b;
    }

    /*
     * Not using the HOMAR library. This is primarily used for autonomous.
     */
    public void drive(double yIn, double xIn)
    {
        for(int i = 0; i < 2; i++)
        {
            driveTrainRightMotors[i].setPower(yIn + xIn);
            driveTrainLeftMotors[i].setPower(yIn - xIn);
        }
    }

    /*
     * Using the HOMAR library. Testing things.
     */
    public void drive(Gamepad gamepad1)
    {
        drivetrain.setVelocity(absMax(-gamepad1.left_stick_y, -gamepad1.right_stick_y));
        drivetrain.setRotation(absMax(gamepad1.left_stick_x, gamepad1.right_stick_x));
    }


    /*
     * drive using encoders.
     */
    public void driveToPos(double forwardPos, double lateralPos)
    {
        for( int i = 0; i< 2; i++)
        {
            driveTrainLeftMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveTrainRightMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            driveTrainRightMotors[i].setTargetPosition((int)(driveTrainRightMotors[i].getCurrentPosition() + (forwardPos + lateralPos)));
            driveTrainLeftMotors[i].setTargetPosition((int)(driveTrainRightMotors[i].getCurrentPosition() + (forwardPos - lateralPos)));
        }


    }


    /*
     * Use the IMU to rotate the robot to a specific angle.
     */
    public void rotate(double angle, float power)
    {

        resetAngle();

        pidRotate.resetIntegration();
        pidRotate.setTarget(angle);
        pidRotate.input(angle);
        pidRotate.output();

        if(angle < Math.abs(Math.round(getAngle()) + 2))
        {
            drive(0,pidRotate.output());
        } else if (angle > Math.abs(Math.round(getAngle()) + 2))
        {
            drive(0,-pidRotate.output());
        } else
        {
            drive(0,0);
        }
    }

    //region Imu methods for reseting, getting, and checking angle

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /*
     * Changes Height of arm.
     * False is down, and True is up.
     */
    public void changeHeight(ToggleBoolean toggleHeight)
    {
        armHeight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(toggleHeight.output())
        {

            armHeight.setTargetPosition(armHeightPos[1]);

            armHeight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armHeight.setPower(armHeightPower);

            while(armHeight.isBusy()) {}

            armHeight.setPower(0);

        } else {
            armHeight.setTargetPosition(armHeightPos[0]);

            armHeight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armHeight.setPower(-armHeightPower);

            while(armHeight.isBusy()) {}

            armHeight.setPower(0);
        }
    }

    public void changeExtension(ToggleBoolean toggleExtension)
    {
        armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(toggleExtension.output())
        {

            armExtension.setTargetPosition(armExtensionPos[1]);

            armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armExtension.setPower(armExtensionPower);

            while(armExtension.isBusy()) {}

            armExtension.setPower(0);

        } else {
            armExtension.setTargetPosition(armExtensionPos[0]);

            armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armExtension.setPower(-armExtensionPower);

            while(armExtension.isBusy()) {}

            armExtension.setPower(0);
        }
    }


 }

