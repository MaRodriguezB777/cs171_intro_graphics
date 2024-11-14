// Fragment Shader
varying vec4 point;
varying vec3 n;
uniform int nLights;  // Number of lights
// uniform vec3 camPos;

void main()
{
    // At the end, convert colors to vec4(_, _, _, 1.0)
    vec4 ambColor = gl_LightModel.ambient * gl_FrontMaterial.ambient;
    vec3 diffuseSum = vec3(0, 0, 0);
    vec3 specularSum = vec3(0, 0, 0);
    vec3 v = point.xyz;
    vec3 camDir = normalize(-v);

    // Loop over all lights
    for (int i = 0; i < nLights; ++i)
    {
        // Access the light source
        gl_LightSourceParameters light = gl_LightSource[i];

        // In our code, the ambient, specular, and diffuse is set as the same
        vec3 lColor = light.ambient.xyz;
        vec3 lPos = light.position.xyz;
        vec3 lDir = normalize(lPos - v);

        float d = length(v - lPos);
        float attenuation = 1.0 / (1.0 + light.quadraticAttenuation * d * d);
        lColor *= attenuation;

        vec3 lightDiffuse = lColor * max(0.0, dot(n, lDir));
        diffuseSum += lightDiffuse;

        float specVal = max(0.0, dot(n, normalize(camDir + lDir)));
        vec3 lightSpecular = lColor * pow(specVal, gl_FrontMaterial.shininess);
        specularSum += lightSpecular;
    }

    // Accumulate the color components
    vec4 diffuseSum4v = vec4(diffuseSum.x, diffuseSum.y, diffuseSum.z, 1.0);
    vec4 specularSum4v = vec4(specularSum.x, specularSum.y, specularSum.z, 1.0);
    vec4 result = ambColor + diffuseSum4v * gl_FrontMaterial.diffuse + specularSum4v * gl_FrontMaterial.specular;
    vec4 ones = vec4(1.0, 1.0, 1.0, 1.0);
    vec4 newColor = min(ones, result);

    // Output the final color
    gl_FragColor = newColor;
    // gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);
}