package frc.subsystem.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
//import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import static frc.robot.Constants.ArmPosition.USE_ARM_ENCODER;
import static frc.robot.Constants.Ports.*;


public class ArmIOTalonFX implements ArmIO {

    private final TalonFX leadTalonFX;
    //private final TalonFX followTalonFX;
    private PhoenixPIDController leadTalonFXPIDController;
    private final CANcoder absoluteEncoder;


    private final StatusSignal<Double> leadPosition;
    private final StatusSignal<Double> leadVelocity;
    private final StatusSignal<Double> leadCurrent;
    private final StatusSignal<Double> leadTemp;
    private final StatusSignal<Double> leadVoltage;
//    private final StatusSignal<Double> followVelocity;
//    private final StatusSignal<Double> followPosition;
//    private final StatusSignal<Double> followTemp;
//    private final StatusSignal<Double> followVoltage;
//    private final StatusSignal<Double> followBusVoltage;
//    private final StatusSignal<Double> followCurrent;
    private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);



    public ArmIOTalonFX() {

        leadTalonFX = new TalonFX(LEAD_CAN_ID);//second parameter=Canbus
        //followTalonFX = new TalonFX(FOLLOW_CAN_ID);
        absoluteEncoder = new CANcoder(ABSOLUTE_CAN_ID);

        var talonFXConfigs = new TalonFXConfiguration();

        var armFeedBackConfigs = talonFXConfigs.Feedback;
        armFeedBackConfigs.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();     //TODO change values for line 42,44,45
        armFeedBackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        armFeedBackConfigs.SensorToMechanismRatio = 1;
        armFeedBackConfigs.RotorToSensorRatio = 1.0 / 144;

        PhoenixPIDController leadTalonFXPIDController = new PhoenixPIDController(1, 2, 3);
        var armMotionMagicConfig = talonFXConfigs.MotionMagic;
        armMotionMagicConfig.MotionMagicAcceleration = 100;     //TODO change motion magic values
        armMotionMagicConfig.MotionMagicCruiseVelocity = 50;
        armMotionMagicConfig.MotionMagicJerk = 50;


        TalonFXConfiguration cfg = new TalonFXConfiguration();

        /* Configure current limits */
        MotionMagicConfigs pivot_current = cfg.MotionMagic;
        pivot_current.MotionMagicCruiseVelocity = 1; // 5 rotations per second cruise
        pivot_current.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel
        pivot_current.MotionMagicJerk = 50;

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = 0;
        slot0.kI = 0;
        slot0.kD = 0;
        slot0.kV = 0;
        slot0.kS = 0; // Approximately 0.25V to get the mechanism moving
        slot0.kG = 0;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        leadTalonFX.getConfigurator().apply(cfg);
//        followTalonFX.getConfigurator().apply(cfg);
//        followTalonFX.setControl(new Follower(leadTalonFX.getDeviceID(), false));


        var followCurrentLimitsConfigs = talonFXConfigs.CurrentLimits;
//        followTalonFX.clearStickyFault_SupplyCurrLimit(Constants.ARM_SMART_CURRENT_LIMIT);
        var closed = talonFXConfigs.ClosedLoopRamps;
        closed.VoltageClosedLoopRampPeriod = 0.75;

//        followTalonFX.getConfigurator().apply(closed);
//        followTalonFX.setNeutralMode(NeutralModeValue.Brake);


        leadPosition = leadTalonFX.getPosition();
        leadVelocity = leadTalonFX.getVelocity();
        leadCurrent = leadTalonFX.getSupplyCurrent();
        leadTemp = leadTalonFX.getDeviceTemp();
        leadVoltage = leadTalonFX.getMotorVoltage();

//        followPosition = followTalonFX.getPosition();
//        followVelocity = followTalonFX.getVelocity();
//        followCurrent = followTalonFX.getSupplyCurrent();
//        followTemp = followTalonFX.getDeviceTemp();
//        followVoltage = followTalonFX.getMotorVoltage();
//        followBusVoltage = followTalonFX.getSupplyVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(4, leadPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(4, leadVelocity, leadVoltage);

        leadTalonFX.optimizeBusUtilization();
//        followTalonFX.optimizeBusUtilization();
    }

    public void setPosition(double position){
        leadTalonFX.setControl(motionMagicControl.withPosition(position).withSlot(0));
    }

    public void updateInputs(ArmInputs inputs) {
        BaseStatusSignal.refreshAll(leadPosition, leadVelocity, leadCurrent,
                leadTemp, leadVoltage);

            inputs.leadPosition = leadPosition.getValue();
            inputs.leadVelocity = leadVelocity.getValue();;

            inputs.leadCurrent = leadCurrent.getValue();
            inputs.leadTemp = leadTemp.getValue();
            inputs.leadVoltage =leadVoltage.getValue();

//            inputs.followPosition = followPosition.getValue();
//            inputs.followVelocity = followVelocity.getValue();
//            inputs.followCurrent = followCurrent.getValue();
//            inputs.followTemp = followTemp.getValue();
//            inputs.followVoltage = followVoltage.getValue();
//            inputs.followBusVoltage = followBusVoltage.getValue();;

            if (USE_ARM_ENCODER){
                assert absoluteEncoder != null;
                inputs.followAbsolutePosition = absoluteEncoder.getPosition().getValue();
            }
    }

    @Override
    public void setLeadVoltage(double voltage) {

            leadTalonFX.setVoltage(voltage);
    }


    public void setLeadVPosition(double position, double arbFFVoltage) {
            leadTalonFX.getConfigurator().setPosition(position);
    }


//    public void setFollowVoltage(double voltage) {
//        followTalonFX.setVoltage(voltage);
//    }


    public void resetLeadVPosition(double position) {
        absoluteEncoder.setPosition(position);
    }

//    public void resetFollowPosition(double position) {
//        followAbsoluteEncoder.setPosition(position);
//    }

    
    public void setAutoGrab(boolean enabled) {
       //TODO

    }
}