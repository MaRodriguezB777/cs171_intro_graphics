/* Straightforward vertex shader that will calculate per-vertex normals
 * if toggled on and use the normal values for the texture coordinates
 * of the sky and leaf textures, passing them to the fragment shader.
 * This also calculates the vertex positions, using a cosine function
 * of x, y, and t to calculate z.
 */

varying vec2 texCoord;
varying mat3 TBN;

void main()
{
    // Pass texture coordinates to fragment shader
    texCoord = gl_MultiTexCoord0.st;

    // Compute TBN matrix for a flat quad
    vec3 normal = vec3(0.0, 0.0, 1.0);
    vec3 tangent = vec3(1.0, 0.0, 0.0);
    vec3 bitangent = vec3(0.0, 1.0, 0.0);

    // Create TBN matrix
    TBN = mat3(tangent, bitangent, normal);

    // Transform vertex position
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
