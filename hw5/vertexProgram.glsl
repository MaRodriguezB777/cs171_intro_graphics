/* Don't need to do anything, just pass point info:
*/

// Vertex Shader
varying vec4 point;
varying vec3 n;

void main()
{
    // Transform the normal to camera space
    point = gl_ModelViewMatrix * gl_Vertex;
    n = normalize(gl_NormalMatrix * gl_Normal);

    // Transform the vertex position to clip space
    gl_Position = ftransform();
}