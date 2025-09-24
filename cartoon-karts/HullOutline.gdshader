// Complete 3D Toon Shader with Cel Shading and Outlines
// Save as: toon_shader.gdshader

shader_type spatial;
render_mode blend_mix, depth_draw_opaque, cull_disabled, diffuse_lambert, specular_disabled;

// Basic Material Properties
uniform vec4 base_color : source_color = vec4(1.0, 1.0, 1.0, 1.0);
uniform sampler2D albedo_texture : source_color, hint_default_white;
uniform float metallic : hint_range(0.0, 1.0) = 0.0;
uniform float roughness : hint_range(0.0, 1.0) = 0.8;

// Toon Shading Controls
uniform int shade_levels : hint_range(2, 10) = 4;
uniform float shade_smoothing : hint_range(0.001, 0.1) = 0.02;
uniform vec4 shadow_color : source_color = vec4(0.6, 0.6, 0.8, 1.0);
uniform float shadow_strength : hint_range(0.0, 1.0) = 0.7;

// Rim Lighting
uniform float rim_threshold : hint_range(0.0, 2.0) = 0.8;
uniform float rim_smoothing : hint_range(0.001, 0.1) = 0.05;
uniform vec4 rim_color : source_color = vec4(1.0, 1.0, 1.0, 1.0);
uniform float rim_strength : hint_range(0.0, 3.0) = 1.5;

// Specular Highlights
uniform float specular_threshold : hint_range(0.0, 1.0) = 0.85;
uniform float specular_smoothing : hint_range(0.001, 0.1) = 0.05;
uniform vec4 specular_color : source_color = vec4(1.0, 1.0, 1.0, 1.0);
uniform float specular_strength : hint_range(0.0, 3.0) = 1.0;

// Outline Controls
uniform bool enable_outlines = true;
uniform vec4 outline_color : source_color = vec4(0.0, 0.0, 0.0, 1.0);
uniform float outline_width : hint_range(0.0, 0.05) = 0.01;
uniform bool outline_use_vertex_colors = false;

// Advanced Lighting
uniform float ambient_strength : hint_range(0.0, 1.0) = 0.1;
uniform vec4 ambient_color : source_color = vec4(0.4, 0.6, 1.0, 1.0);

varying vec3 world_position;
varying vec3 world_normal;
varying vec3 view_direction;
varying float is_outline;

void vertex() {
    world_position = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
    world_normal = normalize((MODEL_MATRIX * vec4(NORMAL, 0.0)).xyz);
    view_direction = normalize(CAMERA_POSITION_WORLD - world_position);
    
    // Determine if this is an outline pass based on face culling
    // We use the dot product of normal and view direction
    float facing = dot(world_normal, view_direction);
    is_outline = 0.0;
    
    // If outlines are enabled and we're rendering back faces
    if (enable_outlines && facing < 0.0) {
        is_outline = 1.0;
        // Push vertices outward for outline effect
        VERTEX += NORMAL * outline_width;
        // Flip normal for proper lighting on back faces
        NORMAL = -NORMAL;
    }
}

float toonify(float value, int levels, float smoothing) {
    float level_size = 1.0 / float(levels);
    float level = floor(value / level_size) * level_size;
    return smoothstep(level - smoothing, level + smoothing, value);
}

vec3 calculate_toon_lighting(vec3 albedo, vec3 normal, vec3 view_dir) {
    // Get the main light direction and color
    vec3 light_dir = normalize(-LIGHT_DIRECTION);
    vec3 light_color = LIGHT_COLOR;
    float light_energy = LIGHT_ENERGY;
    
    // Calculate basic diffuse lighting
    float NdotL = max(dot(normal, light_dir), 0.0);
    float diffuse_intensity = NdotL * light_energy;
    
    // Apply toon shading quantization
    float toon_diffuse = toonify(diffuse_intensity, shade_levels, shade_smoothing);
    
    // Mix between shadow color and full lighting
    vec3 diffuse_result = mix(
        albedo * shadow_color.rgb * shadow_strength,
        albedo * light_color,
        toon_diffuse
    );
    
    // Add ambient lighting
    vec3 ambient = albedo * ambient_color.rgb * ambient_strength;
    diffuse_result += ambient;
    
    // Calculate rim lighting
    float rim = 1.0 - max(dot(view_dir, normal), 0.0);
    rim = smoothstep(rim_threshold - rim_smoothing, rim_threshold + rim_smoothing, rim);
    diffuse_result += rim_color.rgb * rim * rim_strength;
    
    // Calculate toon specular
    vec3 reflect_dir = reflect(-light_dir, normal);
    float spec = max(dot(view_dir, reflect_dir), 0.0);
    spec = smoothstep(specular_threshold - specular_smoothing, specular_threshold + specular_smoothing, spec);
    diffuse_result += specular_color.rgb * spec * specular_strength * light_energy;
    
    return diffuse_result;
}

void fragment() {
    // Handle outline rendering
    if (is_outline > 0.5) {
        if (outline_use_vertex_colors) {
            ALBEDO = COLOR.rgb * outline_color.rgb;
        } else {
            ALBEDO = outline_color.rgb;
        }
        ALPHA = outline_color.a;
        return;
    }
    
    // Main surface rendering
    vec4 tex_color = texture(albedo_texture, UV);
    vec3 albedo = base_color.rgb * tex_color.rgb;
    
    vec3 normal = normalize(world_normal);
    vec3 view_dir = normalize(view_direction);
    
    // Calculate toon lighting
    vec3 final_color = calculate_toon_lighting(albedo, normal, view_dir);
    
    // Output final color
    ALBEDO = final_color;
    METALLIC = metallic;
    ROUGHNESS = roughness;
    ALPHA = base_color.a * tex_color.a;
}