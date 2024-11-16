/* Straightforward fragment shader that will calculate per-fragment
 * normals if toggled on and use the normal values for the texture
 * coordinates of the sky and leaf textures, else it will take the
 * texture coordinates from the vertex shader. The two textures are
 * then mixed together and displayed.
 */

varying vec2 texCoord;
varying mat3 TBN;

uniform sampler2D colorMap, normalMap;

void main()
{
    // Sample the normal from the normal map and convert to [-1, 1]
    vec3 normal = texture2D(normalMap, texCoord).rgb * 2.0 - 1.0;
    normal = normalize(TBN * normal);

    // Define a simple light direction (e.g., coming from the viewer)
    vec3 lightDir = normalize(vec3(0.0, 0.0, 1.0));

    // Calculate diffuse lighting
    float diff = max(dot(normal, lightDir), 0.0);

    // Sample the color map
    vec4 color = texture2D(colorMap, texCoord);

    // Apply the lighting to the color
    gl_FragColor = vec4(color.rgb * diff, color.a);
}
