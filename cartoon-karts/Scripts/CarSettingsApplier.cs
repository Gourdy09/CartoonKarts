using Godot;
using System;
using System.Collections.Generic;

public partial class CarSettingsApplier : Node
{
    public override void _Ready()
    {
        GD.Print("CarSettingsApplier: Initializing...");
        
        // Small delay to ensure all nodes are fully loaded
        GetTree().CreateTimer(0.1).Timeout += ApplySettings;
    }
    
    private void ApplySettings()
    {
        var settings = LoadSettings();
        if (settings.Count == 0)
        {
            GD.Print("CarSettingsApplier: No tuning settings found, using defaults");
            return;
        }
        
        GD.Print($"CarSettingsApplier: Found {settings.Count} settings to apply");
        
        // Find the Car root node (we're attached to it)
        Node carRoot = GetParent();
        
        // Try to find Player node
        var playerNode = carRoot.GetNodeOrNull("Player");
        if (playerNode == null)
        {
            GD.PrintErr("CarSettingsApplier: Player node not found!");
            GD.Print($"Available children of {carRoot.Name}: ");
            foreach (var child in carRoot.GetChildren())
            {
                GD.Print($"  - {child.Name}");
            }
            return;
        }
        
        GD.Print("CarSettingsApplier: Found Player node");
        
        // Apply Engine settings
        var engine = playerNode.GetNodeOrNull<CarEngine>("Engine");
        if (engine != null)
        {
            ApplyEngineSettings(engine, settings);
        }
        else
        {
            GD.PrintErr("CarSettingsApplier: Engine node not found under Player!");
        }
        
        // Apply Differential settings
        var differential = playerNode.GetNodeOrNull<Differential>("Differential");
        if (differential != null)
        {
            ApplyDifferentialSettings(differential, settings);
        }
        else
        {
            GD.PrintErr("CarSettingsApplier: Differential node not found under Player!");
        }
        
        // Apply Chassis settings
        var chassis = playerNode.GetNodeOrNull<RigidBody3D>("Chassis");
        if (chassis != null)
        {
            ApplyChassisSettings(chassis, settings);
        }
        else
        {
            GD.PrintErr("CarSettingsApplier: Chassis node not found under Player!");
        }
        
        // Apply Wheel settings to all wheels
        string[] wheels = {"FL", "FR", "BL", "BR"};
        foreach (var wheelName in wheels)
        {
            var wheel = playerNode.GetNodeOrNull<Wheel>(wheelName);
            if (wheel != null)
            {
                ApplyWheelSettings(wheel, settings);
            }
            else
            {
                GD.PrintErr($"CarSettingsApplier: Wheel {wheelName} not found under Player!");
            }
        }
        
        GD.Print("CarSettingsApplier: All settings applied successfully!");
    }
    
    private void ApplyEngineSettings(CarEngine engine, Dictionary<string, float> settings)
    {
        if (settings.ContainsKey("MaxRPM"))
        {
            engine.maxEngineRPM = settings["MaxRPM"];
            GD.Print($"✓ Applied MaxRPM: {settings["MaxRPM"]}");
        }
        
        if (settings.ContainsKey("PeakTorque"))
        {
            engine.customPeakTorque = settings["PeakTorque"];
            GD.Print($"✓ Applied PeakTorque: {settings["PeakTorque"]}");
        }
        
        if (settings.ContainsKey("PeakTorqueRPM"))
        {
            engine.customPeakTorqueRPM = settings["PeakTorqueRPM"];
            GD.Print($"✓ Applied PeakTorqueRPM: {settings["PeakTorqueRPM"]}");
        }
    }
    
    private void ApplyDifferentialSettings(Differential differential, Dictionary<string, float> settings)
    {
        // Load drive type from file (it's stored as string)
        var saveFile = FileAccess.Open("user://car_tuning.save", FileAccess.ModeFlags.Read);
        if (saveFile != null)
        {
            while (!saveFile.EofReached())
            {
                string line = saveFile.GetLine().Trim();
                if (line.StartsWith("DriveType="))
                {
                    string driveType = line.Split('=')[1];
                    differential.driveTrainType = driveType;
                    GD.Print($"✓ Applied DriveType: {driveType}");
                    break;
                }
            }
            saveFile.Close();
        }
    }
    
    private void ApplyChassisSettings(RigidBody3D chassis, Dictionary<string, float> settings)
    {
        if (settings.ContainsKey("ChassisMass"))
        {
            chassis.Mass = settings["ChassisMass"];
            GD.Print($"✓ Applied Chassis Mass: {settings["ChassisMass"]} kg");
        }
    }
    
    private void ApplyWheelSettings(Wheel wheel, Dictionary<string, float> settings)
    {
        int appliedCount = 0;
        
        if (settings.ContainsKey("MaxSteerAngle"))
        {
            wheel.maxSteerAngle = settings["MaxSteerAngle"];
            appliedCount++;
        }
            
        if (settings.ContainsKey("LateralGrip"))
        {
            wheel.lateralGrip = settings["LateralGrip"];
            appliedCount++;
        }
            
        if (settings.ContainsKey("RollingResistance"))
        {
            wheel.rollingResistance = settings["RollingResistance"];
            appliedCount++;
        }
            
        if (settings.ContainsKey("BrakeForce"))
        {
            wheel.brakeForce = settings["BrakeForce"];
            appliedCount++;
        }
            
        if (settings.ContainsKey("SuspensionForce"))
        {
            wheel.suspensionForce = settings["SuspensionForce"];
            appliedCount++;
        }
            
        if (settings.ContainsKey("SuspensionDamping"))
        {
            wheel.suspensionDamping = settings["SuspensionDamping"];
            appliedCount++;
        }
            
        if (settings.ContainsKey("AntiRollForce"))
        {
            wheel.antiRollForce = settings["AntiRollForce"];
            appliedCount++;
        }
            
        if (settings.ContainsKey("WheelMass"))
        {
            wheel.Mass = settings["WheelMass"];
            appliedCount++;
        }
        
        if (appliedCount > 0)
        {
            GD.Print($"✓ Applied {appliedCount} settings to wheel: {wheel.Name}");
        }
    }
    
    private Dictionary<string, float> LoadSettings()
    {
        var settings = new Dictionary<string, float>();
        
        var saveFile = FileAccess.Open("user://car_tuning.save", FileAccess.ModeFlags.Read);
        if (saveFile == null)
        {
            GD.Print("CarSettingsApplier: No save file found at user://car_tuning.save");
            return settings;
        }
        
        GD.Print("CarSettingsApplier: Loading settings from file...");
        
        while (!saveFile.EofReached())
        {
            string line = saveFile.GetLine().Trim();
            if (string.IsNullOrEmpty(line)) continue;
            
            var parts = line.Split('=');
            if (parts.Length != 2) continue;
            
            string key = parts[0];
            string value = parts[1];
            
            // Skip DriveType (handled separately)
            if (key == "DriveType") 
            {
                GD.Print($"  Found DriveType: {value}");
                continue;
            }
            
            if (float.TryParse(value, out float floatValue))
            {
                settings[key] = floatValue;
                GD.Print($"  Loaded {key} = {floatValue}");
            }
        }
        saveFile.Close();
        
        return settings;
    }
}