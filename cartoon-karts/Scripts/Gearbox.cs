using Godot;
using System;

public partial class Gearbox : Node
{
    private CarEngine engine;
    private RigidBody3D chassis; // Reference to get vehicle speed
    private int currentGear = 0; // Start in first gear
    private float[] gearRatios = {1f, 1.3f, 1.9f, 2.1f, 2.6f}; // 5-speed transmission
    private float[] shiftUpSpeeds = {15f, 25f, 40f, 60f}; // km/h shift points
    private float[] shiftDownSpeeds = {10f, 20f, 35f, 55f}; // km/h downshift points
    private float reverseGearRatio = 4.0f;
    private float shiftDelay = 0.3f; // Seconds between shifts
    private float timeSinceLastShift = 0f;

    [Export] public float drivetrainTorque { get; private set; }
    [Export] public int currentGearDisplay { get; private set; } // For UI display

    public override void _Ready()
    {
        engine = GetParent().GetNode("Engine") as CarEngine;
        chassis = GetParent().GetNode("Chassis") as RigidBody3D;
    }

    public override void _PhysicsProcess(double delta)
    {
        timeSinceLastShift += (float)delta;
        
        if (engine.isReversing)
        {
            // Reverse gear
            drivetrainTorque = -engine.engineTorque * reverseGearRatio;
            currentGearDisplay = -1; // R
        }
        else if (engine.isNeutral)
        {
            // Neutral - no torque transmission
            drivetrainTorque = 0;
            currentGearDisplay = 0; // N
            // Don't do any shifting logic in neutral
        }
        else
        {
            // Forward gears with automatic shifting
            handleAutomaticShifting();
            drivetrainTorque = engine.engineTorque * gearRatios[currentGear];
            currentGearDisplay = currentGear + 1; // Display as 1-5 instead of 0-4
        }
    }

    private void handleAutomaticShifting()
    {
        if (timeSinceLastShift < shiftDelay) return; // Prevent rapid shifting
        
        float speed = chassis.LinearVelocity.Length() * 3.6f; // Convert m/s to km/h
        float throttle = PlayerInput.Instance.throttle;
        
        // NEUTRAL FIX: Auto-select first gear when accelerating from standstill
        if (speed < 5f && throttle > 0.1f && currentGear > 0)
        {
            currentGear = 0; // Drop to first gear for acceleration
            timeSinceLastShift = 0f;
            GD.Print("Auto-selected 1st gear for acceleration");
            return;
        }
        
        // Shift up conditions
        if (currentGear < gearRatios.Length - 1 && speed > shiftUpSpeeds[currentGear])
        {
            // Shift up more aggressively with more throttle
            float shiftThreshold = shiftUpSpeeds[currentGear] * (1.0f - throttle * 0.3f);
            if (speed > shiftThreshold)
            {
                currentGear++;
                timeSinceLastShift = 0f;
                GD.Print($"Shifted up to gear {currentGear + 1}");
            }
        }
        // Shift down conditions
        else if (currentGear > 0 && speed < shiftDownSpeeds[currentGear])
        {
            // Only downshift if we're really slowing down or need more torque
            if (speed < shiftDownSpeeds[currentGear] * 0.9f || throttle > 0.8f)
            {
                currentGear--;
                timeSinceLastShift = 0f;
                GD.Print($"Shifted down to gear {currentGear + 1}");
            }
        }
    }
}