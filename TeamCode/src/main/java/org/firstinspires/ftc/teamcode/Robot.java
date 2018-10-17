package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import edu.spa.ftclib.internal.drivetrain.TankDrivetrain;

public class Robot
{
    /* Public OpMode members. */
    public DcMotor leftFrontDrive   = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive  = null;
    public DcMotor rightBackDrive = null;

    /* local OpMode members. */
    private HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    private TankDrivetrain drivetrain;

    private DcMotor[] driveTrainLeftMotors;
    private DcMotor[] driveTrainRightMotors;

    private BNO055IMU imu = null;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    /* Constructor */
    public Robot(HardwareMap hwMap){
        this.hwMap = hwMap;
    }

    public void init()
    {
        leftFrontDrive = hwMap.get(DcMotor.class, "left_front");
        leftBackDrive = hwMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front");
        rightBackDrive = hwMap.get(DcMotor.class, "right_back");
        driveTrainLeftMotors = new DcMotor[]{leftBackDrive, leftFrontDrive};
        driveTrainRightMotors = new DcMotor[]{rightBackDrive, rightFrontDrive};

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

    public void drive(float yIn, float xIn)
    {
        for(int i = 0; i < 2; i++)
        {
            driveTrainRightMotors[i].setPower(yIn + xIn);
            driveTrainLeftMotors[i].setPower(yIn - xIn);
        }
    }

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

    public void rotate(double angle, float power)
    {

        if(angle < Math.abs(Math.round(getAngle()) + 2))
        {
            drive(0,power);
        } else if (angle > Math.abs(Math.round(getAngle()) + 2))
        {
            drive(0,-power);
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
 }

