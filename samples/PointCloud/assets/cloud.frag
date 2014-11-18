#version 150 core

uniform sampler2D	uTextureColor;
uniform sampler2D	uTextureDepthToColorTable;

in float			vDepth;
in vec2				vTexCoord0;

out vec4			gl_FragColor;

void main( void )
{
	if ( vDepth <= 0.0 || vDepth >= 4096.0 ) {
		discard;
	}
	vec2 uv			= texture( uTextureDepthToColorTable, vTexCoord0 ).rg;

	gl_FragColor	= texture( uTextureColor, uv );
}
 