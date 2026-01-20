// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Motor Configurations */
public class MotorConfigs {
    public static CurrentLimitsConfigs getCurrentLimitConfig(String motorType) {
        switch (motorType) {
            case "KrakenX60":
                return new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(40)
                        .withStatorCurrentLimitEnable(true);
            case "Falcon500":
                return new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(60)
                        .withStatorCurrentLimitEnable(true);
            default:
                return null;
        }
    }

    public static MotorOutputConfigs getMotorOutputConfigs(NeutralModeValue neutralModeValue,
            InvertedValue invertedValue) {
        return new MotorOutputConfigs()
                .withInverted(invertedValue)
                .withNeutralMode(neutralModeValue);
    }

    public static Slot0Configs getSlot0Configs(double kP, double kI, double kD, double kS, double kV, double kA,
            double kG) {
        return new Slot0Configs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(kS)
                .withKV(kV)
                .withKA(kA)
                .withKG(kG);
    }

    public static FeedbackConfigs getFeedbackConfigs(double SensorToMechanismRatio) {
        return new FeedbackConfigs()
                .withSensorToMechanismRatio(SensorToMechanismRatio);
    }

    public static MotionMagicConfigs geMotionMagicConfigs(double MotionMagicAcceleration,
            double MotionMagicCruiseVelocity, double MotionMagicJerk) {
        return new MotionMagicConfigs()
                .withMotionMagicAcceleration(MotionMagicAcceleration)
                .withMotionMagicCruiseVelocity(MotionMagicCruiseVelocity)
                .withMotionMagicJerk(MotionMagicJerk);
    }
}