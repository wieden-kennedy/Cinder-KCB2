uniform mat4		ciModelViewProjection;
uniform usampler2D	uTextureDepth;
uniform sampler2D	uTextureDepthToCameraTable;

in vec2				ciPosition;

out float			vDepth;
out vec2			vTexCoord0;

void main( void )
{
	vTexCoord0	= ciPosition;

	vec2 uv		= vTexCoord0;
	uv.t		= 1.0 - uv.t;

	vDepth		= texture( uTextureDepth, uv ).r;
	vec3 pos	= vec3( texture( uTextureDepthToCameraTable, vTexCoord0 ).rg * vDepth, vDepth );

	gl_Position = ciModelViewProjection * vec4( pos * 0.1, 1.0 );
};
 