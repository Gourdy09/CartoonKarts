using Godot;
using System;

public partial class CarEngine : Node
{
	private float engineRPM = 0;
	[Export] public float engineTorque { private set; get; } = 0;
	[Export] public bool isReversing { private set; get; } = false;
	[Export] public bool isNeutral { private set; get; } = false;
	
	// Tunable parameters - can be modified by tuning menu
	[Export] public float maxEngineRPM = 10000;
	[Export] public float customPeakTorque = 500f;
	[Export] public float customPeakTorqueRPM = 8000f;
	
	private float engineRPMPickup = 1;
	private float reverseMultiplier = 0.4f;
	private float idleRPM = 900f;

	public override void _PhysicsProcess(double delta)
	{
		float throttleInput = PlayerInput.Instance.throttle;
		float reverseInput = PlayerInput.Instance.reverse;
		
		// Determine gear state
		if (reverseInput > 0 && throttleInput <= 0)
		{
			isReversing = true;
			isNeutral = false;
		}
		else if (throttleInput > 0 && reverseInput <= 0)
		{
			isReversing = false;
			isNeutral = false;
		}
		else
		{
			// No input - go to neutral (no torque transmission)
			isNeutral = true;
			isReversing = false;
		}
		
		// Calculate RPM based on input
		float effectiveThrottle = isReversing ? reverseInput : throttleInput;
		calculateRPM(effectiveThrottle, delta);
		
		// Only generate torque when not in neutral
		if (isNeutral)
		{
			engineTorque = 0; // No creep when no input
		}
		else
		{
			// Use custom tuning values
			calculateEngineTorque(engineRPM, customPeakTorque, 50, .7f, customPeakTorqueRPM, 2000);
			
			// Apply reverse multiplier if reversing
			if (isReversing)
			{
				engineTorque *= reverseMultiplier;
			}
		}
	}

	private void calculateEngineTorque(float RPM, float peakTorque, float baseTorque, float curveSteepness, float peakTorqueRPM, float torqueBandWidth)
	{
		if (RPM <= peakTorqueRPM)
		{
			double exponent = -Math.Pow(curveSteepness * (RPM - peakTorqueRPM), 2) / Math.Pow(torqueBandWidth, 2);
			float torque = (peakTorque - baseTorque) * (float)Math.Exp(exponent) + baseTorque;
			engineTorque = MathF.Min(torque, peakTorque);
		}
		else
		{
			engineTorque = peakTorque;
		}
	}

	private void calculateRPM(float throttle, double delta)
	{
		if (throttle > 0)
		{
			engineRPM = (float)Mathf.Lerp(engineRPM, maxEngineRPM, throttle * delta * engineRPMPickup);
		}
		else
		{
			engineRPM = (float)Mathf.Lerp(engineRPM, idleRPM, delta * engineRPMPickup);
			engineRPM = MathF.Max(engineRPM, idleRPM);
		}
	}
}