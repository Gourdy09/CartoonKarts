using Godot;
using System;
using System.Collections.Generic;

public partial class CarTuningMenu : Control
{
    // References to UI elements
    private Button backButton;
    private Button resetButton;
    private Button saveButton;
    
    // Dictionary to store all slider-value pairs
    private Dictionary<string, (HSlider slider, LineEdit value)> tuningControls = new();
    private OptionButton driveTypeOption;
    
    // Default values
    private Dictionary<string, float> defaultValues = new()
    {
        {"MaxRPM", 10000f},
        {"PeakTorque", 500f},
        {"PeakTorqueRPM", 8000f},
        {"MaxSteerAngle", 30f},
        {"LateralGrip", 150f},
        {"RollingResistance", 3f},
        {"BrakeForce", 80f},
        {"SuspensionForce", 1500f},
        {"SuspensionDamping", 400f},
        {"AntiRollForce", 1000f},
        {"ChassisMass", 25f},
        {"WheelMass", 7f}
    };
    
    public override void _Ready()
    {
        // Get button references
        backButton = GetNode<Button>("MarginContainer/VBoxContainer/Header/BackButton");
        resetButton = GetNode<Button>("MarginContainer/VBoxContainer/BottomButtons/ResetButton");
        saveButton = GetNode<Button>("MarginContainer/VBoxContainer/BottomButtons/SaveButton");
        driveTypeOption = GetNode<OptionButton>("MarginContainer/VBoxContainer/ScrollContainer/TuningPanel/DifferentialSection/DriveType/OptionButton");
        
        // Connect buttons
        backButton.Pressed += OnBackPressed;
        resetButton.Pressed += OnResetPressed;
        saveButton.Pressed += OnSavePressed;
        
        // Setup all tuning controls
        SetupTuningControl("EngineSection/MaxRPM");
        SetupTuningControl("EngineSection/PeakTorque");
        SetupTuningControl("EngineSection/PeakTorqueRPM");
        SetupTuningControl("WheelSection/MaxSteerAngle");
        SetupTuningControl("WheelSection/LateralGrip");
        SetupTuningControl("WheelSection/RollingResistance");
        SetupTuningControl("WheelSection/BrakeForce");
        SetupTuningControl("SuspensionSection/SuspensionForce");
        SetupTuningControl("SuspensionSection/SuspensionDamping");
        SetupTuningControl("SuspensionSection/AntiRollForce");
        SetupTuningControl("VehicleSection/ChassisMass");
        SetupTuningControl("VehicleSection/WheelMass");
        
        // Load saved settings
        LoadSettings();
    }
    
    private void SetupTuningControl(string path)
    {
        string basePath = "MarginContainer/VBoxContainer/ScrollContainer/TuningPanel/";
        var container = GetNode<HBoxContainer>(basePath + path);
        var slider = container.GetNode<HSlider>("Slider");
        var valueEdit = container.GetNode<LineEdit>("Value");
        
        // Extract control name from path
        string controlName = path.Split('/')[1];
        tuningControls[controlName] = (slider, valueEdit);
        
        // Connect signals
        slider.ValueChanged += (value) => OnSliderChanged(controlName, value);
        valueEdit.TextSubmitted += (text) => OnValueSubmitted(controlName, text);
    }
    
    private void OnSliderChanged(string controlName, double value)
    {
        var (slider, valueEdit) = tuningControls[controlName];
        
        // Format based on step size
        if (slider.Step >= 1)
            valueEdit.Text = ((int)value).ToString();
        else
            valueEdit.Text = value.ToString("F1");
    }
    
    private void OnValueSubmitted(string controlName, string text)
    {
        var (slider, valueEdit) = tuningControls[controlName];
        
        if (float.TryParse(text, out float value))
        {
            // Clamp to slider range
            value = Mathf.Clamp(value, (float)slider.MinValue, (float)slider.MaxValue);
            slider.Value = value;
            
            // Update text with clamped value
            if (slider.Step >= 1)
                valueEdit.Text = ((int)value).ToString();
            else
                valueEdit.Text = value.ToString("F1");
        }
        else
        {
            // Reset to current slider value if invalid
            if (slider.Step >= 1)
                valueEdit.Text = ((int)slider.Value).ToString();
            else
                valueEdit.Text = slider.Value.ToString("F1");
        }
    }
    
    private void OnBackPressed()
    {
        GetTree().ChangeSceneToFile("res://Scenes/MainMenu.tscn");
    }
    
    private void OnResetPressed()
    {
        foreach (var kvp in tuningControls)
        {
            if (defaultValues.ContainsKey(kvp.Key))
            {
                kvp.Value.slider.Value = defaultValues[kvp.Key];
            }
        }
        driveTypeOption.Selected = 1; // AWD
    }
    
    private void OnSavePressed()
    {
        SaveSettings();
        GetTree().ChangeSceneToFile("res://Scenes/Track1.tscn");
    }
    
    private void SaveSettings()
    {
        // Save to global singleton for access in game
        var settings = new Dictionary<string, Variant>();
        
        foreach (var kvp in tuningControls)
        {
            settings[kvp.Key] = (float)kvp.Value.slider.Value;
        }
        
        // Save drive type
        settings["DriveType"] = driveTypeOption.GetItemText(driveTypeOption.Selected);
        
        // Store in ProjectSettings for global access
        foreach (var kvp in settings)
        {
            ProjectSettings.SetSetting($"tuning/{kvp.Key}", kvp.Value);
        }
        
        // Also save to file for persistence
        var saveFile = FileAccess.Open("user://car_tuning.save", FileAccess.ModeFlags.Write);
        if (saveFile != null)
        {
            foreach (var kvp in settings)
            {
                saveFile.StoreLine($"{kvp.Key}={kvp.Value}");
            }
            saveFile.Close();
            GD.Print("Settings saved successfully!");
        }
        else
        {
            GD.PrintErr("Failed to save settings file!");
        }
    }
    
    private void LoadSettings()
    {
        var saveFile = FileAccess.Open("user://car_tuning.save", FileAccess.ModeFlags.Read);
        if (saveFile != null)
        {
            GD.Print("Loading saved tuning settings...");
            
            while (!saveFile.EofReached())
            {
                string line = saveFile.GetLine().Trim();
                if (string.IsNullOrEmpty(line)) continue;
                
                var parts = line.Split('=');
                if (parts.Length != 2) continue;
                
                string key = parts[0];
                string value = parts[1];
                
                // Handle drive type
                if (key == "DriveType")
                {
                    for (int i = 0; i < driveTypeOption.ItemCount; i++)
                    {
                        if (driveTypeOption.GetItemText(i) == value)
                        {
                            driveTypeOption.Selected = i;
                            GD.Print($"Loaded DriveType: {value}");
                            break;
                        }
                    }
                }
                // Handle numeric values
                else if (tuningControls.ContainsKey(key) && float.TryParse(value, out float floatValue))
                {
                    tuningControls[key].slider.Value = floatValue;
                    GD.Print($"Loaded {key}: {floatValue}");
                }
            }
            saveFile.Close();
        }
        else
        {
            GD.Print("No saved tuning settings found, using defaults");
        }
    }
}