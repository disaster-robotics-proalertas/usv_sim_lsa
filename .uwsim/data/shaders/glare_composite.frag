uniform sampler2DRect osgOcean_ColorBuffer;
uniform sampler2DRect osgOcean_StreakBuffer1;
uniform sampler2DRect osgOcean_StreakBuffer2;
uniform sampler2DRect osgOcean_StreakBuffer3;
uniform sampler2DRect osgOcean_StreakBuffer4;

void main(void)
{
	vec4 fullColor    = texture2DRect(osgOcean_ColorBuffer,   gl_TexCoord[0].st );
	vec4 streakColor1 = texture2DRect(osgOcean_StreakBuffer1, gl_TexCoord[1].st );
	vec4 streakColor2 = texture2DRect(osgOcean_StreakBuffer2, gl_TexCoord[1].st );
	vec4 streakColor3 = texture2DRect(osgOcean_StreakBuffer3, gl_TexCoord[1].st );
	vec4 streakColor4 = texture2DRect(osgOcean_StreakBuffer4, gl_TexCoord[1].st );

	vec4 streak = streakColor1+streakColor2+streakColor3+streakColor4;

	gl_FragColor = vec4( streak.rgb+fullColor.rgb, 1.0);;
}