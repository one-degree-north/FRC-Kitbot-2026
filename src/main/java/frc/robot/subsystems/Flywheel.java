package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConfigs;

public class Flywheel extends SubsystemBase {
    // Constants for PID/Feedforward
    private final double KP = 0.0;
    private final double KI = 0.0;
    private final double KD = 0.0;
    private final double KS = 0.2;
    private final double KV = 0.1;

    private final String CAN_BUS = "rio";
    private final String MOTOR_TYPE = "Falcon500";
    private final double TOLERANCE = 2.0;
    private double targetRPS = 0.0; // Positive = Counter Clockwise, Negative = Clockwise, initially at rest

    private TalonFX m_holder, m_supporter;
    private VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    public Flywheel() {
        this.m_supporter = new TalonFX(1, CAN_BUS);
        this.m_holder = new TalonFX(2, CAN_BUS);
        configureMotors();
    }

    private void configureMotors() {
        var motorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(MotorConfigs.getCurrentLimitConfig(MOTOR_TYPE))
                .withMotorOutput(
                        MotorConfigs.getMotorOutputConfigs(NeutralModeValue.Coast,
                                InvertedValue.CounterClockwise_Positive))
                .withFeedback(MotorConfigs.getFeedbackConfigs(1 / 1));

        var slot0Configs = motorConfigs.Slot0;
        slot0Configs.kP = KP; // Proportional gain
        slot0Configs.kI = KI; // Integral gain
        slot0Configs.kD = KD; // Derivative gain
        slot0Configs.kS = KS; // Static feedforward in volts
        slot0Configs.kV = KV; // Velocity feedforward (Volts / RPS)

        m_holder.getConfigurator().apply(motorConfigs);
        m_supporter.getConfigurator().apply(motorConfigs);
        m_supporter.setControl(new Follower(m_holder.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    /**
     * Get the velocity of the 2 motor sin Revolution per Second
     * 
     * @return the velocity of the 2 motors
     */
    private double getVelocityRPS() {
        return m_holder.getVelocity().getValueAsDouble();
    }

    /**
     * Set the control of the 2 motors
     * 
     * @param req - the control request
     */
    private void setControl(ControlRequest req) {
        if (m_holder.isAlive()) {
            m_holder.setControl(req);
        }
    }

    /**
     * Check if the current motors' RPS is within the tolerance range compare to the
     * targetRPS
     * 
     * @return true if the motors are in the range, false otherwise
     */
    private boolean atSetpoint() {
        return Math.abs(getVelocityRPS() - targetRPS) <= TOLERANCE;
    }

    /**
     * Check if the current motors' RPS is within the tolerance range compare to the
     * targetRPS
     * 
     * @param withTolerance - whether to include tolerance or not
     * @return true if the motors are within the expected range or matches with the
     *         setpoint value
     */
    public boolean atSetpoint(boolean withTolerance) {
        if (withTolerance)
            return atSetpoint();
        return Double.compare(getVelocityRPS(), targetRPS) == 0;
    }

    /**
     * Set the Revolution-per-Second value of the 2 motors
     * 
     * @param RPS - The Revolution-per-Second value in decimals
     */
    public void setTargetRPS(double RPS) {
        targetRPS = RPS;
    }

    @Override
    public void periodic() {
        // Update the motor's velocity
        if (!atSetpoint()) {
            m_velocityRequest.Velocity = targetRPS;
            setControl(m_velocityRequest);
        }

        // Display important information on SmartDashboard
        SmartDashboard.putNumber("Motor Velocity (RPS)", getVelocityRPS());
        SmartDashboard.putNumber("Target Velocity (RPS)", targetRPS);
        SmartDashboard.putNumber("KP Value", KP);
        SmartDashboard.putNumber("KI Value", KI);
        SmartDashboard.putNumber("KD Value", KD);
        SmartDashboard.putNumber("KS Value", KS);
        SmartDashboard.putNumber("KV Value", KV);
    }
}
