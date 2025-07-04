#[compute]
#version 450

#define PI 3.141592

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

layout(binding = 0) uniform sampler2D input_data_image;
layout(binding = 1) uniform sampler2D input_color_image;
layout(rgba16f, binding = 2) uniform image2D reflections_sample;
layout(rgba16f, binding = 3) uniform image2D color_image;
layout(binding = 4) uniform sampler2D depth_image;

layout(binding = 5) uniform uniformBuffer {
	mat4 view;
	mat4 prevview;
	mat4 proj;
	mat4 prevproj;

	vec3 extralargenoiseposition;
	float extralargenoisescale;

	vec3 largenoiseposition;
	float cloud_lighting_sharpness;

	vec3 mediumnoiseposition;
	float lighting_step_distance;

	vec3 smallnoiseposition;
	float atmospheric_density;

	vec4 ambientLightColor;
	vec4 ambientGroundLightColor;
	vec4 ambientfogdistancecolor;

	float small_noise_scale;
	float min_step_distance;
	float max_step_distance;
	float lod_bias;

	float cloud_sharpness;
	float directionalLightsCount;
	float reserveda;
	float anisotropy;

	float cloud_floor;
	float cloud_ceiling;
	float max_step_count;
	float max_lighting_step_count;

	float filterIndex;
	float blurPower;
	float blurQuality;
	float curlPower;

	vec2 WindDirection;
	float fogEffectGround;
	float samplePointsCount;

	float pointLightsCount;
	float pointEffectorCount;
	vec2 reservedb;
} genericData;


struct DirectionalLight {
	vec4 direction; //w = shadow sample count
	vec4 color; //a = intensity
};

struct PointLight {
	vec4 position; //w = radius
	vec4 color; //a = intensity
};

struct PointEffector {
	vec3 position; //w = radius
	float radius;

	float power;
	float attenuation;
	vec2 reserved;
};

layout(binding = 6) uniform LightsBuffer {
	DirectionalLight directionalLights[4];
	PointLight pointLights[128];
	PointEffector pointEffectors[64];
};


// Our push constant
layout(push_constant, std430) uniform Params {
    vec2 input_size;
	float resolutionscale;
    float reserved;
} params;

// Helpers
float remap(float value, float min1, float max1, float min2, float max2) {
  return min2 + (value - min1) * (max2 - min2) / (max1 - min1);
}

float w0(float a)
{
    return (1.0/6.0)*(a*(a*(-a + 3.0) - 3.0) + 1.0);
}

float w1(float a)
{
    return (1.0/6.0)*(a*a*(3.0*a - 6.0) + 4.0);
}

float w2(float a)
{
    return (1.0/6.0)*(a*(a*(-3.0*a + 3.0) + 3.0) + 1.0);
}

float w3(float a)
{
    return (1.0/6.0)*(a*a*a);
}

// g0 and g1 are the two amplitude functions
float g0(float a)
{
    return w0(a) + w1(a);
}

float g1(float a)
{
    return w2(a) + w3(a);
}

// h0 and h1 are the two offset functions
float h0(float a)
{
    return -1.0 + w1(a) / (w0(a) + w1(a));
}

float h1(float a)
{
    return 1.0 + w3(a) / (w2(a) + w3(a));
}

// Sampling

vec4 texture2D_bicubic(sampler2D tex, vec2 uv, vec2 res)
{
	uv = uv*res + 0.5;
	vec2 iuv = floor( uv );
	vec2 fuv = fract( uv );

	float g0x = g0(fuv.x);
	float g1x = g1(fuv.x);
	float h0x = h0(fuv.x);
	float h1x = h1(fuv.x);
	float h0y = h0(fuv.y);
	float h1y = h1(fuv.y);

	vec2 p0 = (vec2(iuv.x + h0x, iuv.y + h0y) - 0.5) / res;
	vec2 p1 = (vec2(iuv.x + h1x, iuv.y + h0y) - 0.5) / res;
	vec2 p2 = (vec2(iuv.x + h0x, iuv.y + h1y) - 0.5) / res;
	vec2 p3 = (vec2(iuv.x + h1x, iuv.y + h1y) - 0.5) / res;

    return g0(fuv.y) * (g0x * texture(tex, p0)  +
                        g1x * texture(tex, p1)) +
           g1(fuv.y) * (g0x * texture(tex, p2)  +
                        g1x * texture(tex, p3));
}

vec4 radialBlurColor(vec4 startColor, sampler2D colorImage, sampler2D depthImage, vec2 uv, vec2 size, inout float startingDepth, float Directions, float blurVertical, float blurHorizontal, float Quality){
    float Pi = 6.28318530718;
    float count = 1.0;
	float theoreticalMaxCount =  Directions * Quality;
	//float stepLerp = 1.0 / theoreticalMaxCount;
    vec4 Color = startColor;
	//float CurrentDepth = startingDepth;
	vec2 newUV = uv;
    for( float d=0.0; d<Pi; d+=Pi/Directions)
    {
		for(float i=1.0/Quality; i<=1.0; i+=1.0/Quality)
        {
			newUV = uv + vec2(cos(d) * blurHorizontal * i, sin(d) * blurVertical * i);
			Color += texture2D_bicubic(colorImage, newUV, size);
			count += 1.0;
			//float newDepth = texture(depthImage, newUV).g;
			startingDepth = max(startingDepth, texture(depthImage, newUV).g);
			// if (startingDepth - newDepth > genericData.max_step_distance){
			// 	CurrentDepth = max(CurrentDepth, newDepth);
			// 	Color += texture2D_bicubic(colorImage, newUV, size);
			// 	count += 1.0;
			// }
        }
    }
	//startingDepth = CurrentDepth
    Color /= count;
    return Color;
}

vec4 radialBlurData(vec4 startColor, float linear_depth, sampler2D image, vec2 uv, float Directions, float blurVertical, float blurHorizontal, float Quality){
    float Pi = 6.28318530718;
    float count = 1.0;
	//float theoreticalMaxCount =  Directions * Quality;
	//float stepLerp = 1.0 / theoreticalMaxCount;
    vec4 Color = startColor;
	//float originalDepth = Color.r;
	//bool isNear = originalDepth < linear_depth;

	//float meanDistancesFar = 0.0;
    for( float d=0.0; d<Pi; d+=Pi/Directions)
    {
		for(float i=1.0/Quality; i<=1.0; i+=1.0/Quality)
        {
			Color = max(Color, texture(image, uv + vec2(cos(d) * blurHorizontal * i, sin(d) * blurVertical * i)));

			//sampled = texture(image, uv + vec2(cos(d) * blurHorizontal * i, sin(d) * blurVertical * i));
			//Color.rgb += sampled.rgb;
			//Color.a = max(Color.a, sampled.a);
			// if (abs(originalDepth - sampled) < 100.0){
			// 	Color += texture(image, uv + vec2(cos(d) * blurHorizontal * i, sin(d) * blurVertical * i)).r;
			// 	count += 1.0;
			// }

			// if (sampled > linear_depth){
			// 	meanDistancesFar += 1.0;
			// }
			// else{
			// 	meanDistancesFar -= 1.0;
			// }
			// maxDistance = max(maxDistance, sampled);
			// minDistance = min(minDistance, sampled);
			//count += 1.0;
        }
    }
    return Color;
}

// vec4 radialBlurColor(vec4 startColor, sampler2D image, vec2 uv, vec2 size, float Directions, float blurVertical, float blurHorizontal, float Quality){
//     float Pi = 6.28318530718;
//     float count = 1.0;
// 	float theoreticalMaxCount =  Directions * Quality;
// 	//float stepLerp = 1.0 / theoreticalMaxCount;
//     vec4 Color = startColor;
//     for( float d=0.0; d<Pi; d+=Pi/Directions)
//     {
// 		for(float i=1.0/Quality; i<=1.0; i+=1.0/Quality)
//         {
// 			Color += texture2D_bicubic(image, uv + vec2(cos(d) * blurHorizontal * i, sin(d) * blurVertical * i), size);
// 			count += 1.0;
//         }
//     }
//     Color /= count;
//     return Color;
// }


void sampleAtmospherics(
	vec3 curPos,
	float atmosphericHeight,
	float distanceTraveled,
	float Rayleighscaleheight,
	float Miescaleheight,
	vec3 RayleighScatteringCoef,
	float MieScatteringCoef,
	float atmosphericDensity,
	float density,
	inout vec3 totalRlh,
	inout vec3 totalMie,
	inout float iOdRlh,
	inout float iOdMie)
	{
	float iHeight = curPos.y / atmosphericHeight;
	float odStepRlh = exp(-iHeight / Rayleighscaleheight) * distanceTraveled;
	float odStepMie = exp(-iHeight / Miescaleheight) * distanceTraveled;
	iOdRlh += odStepRlh;
	iOdMie += odStepMie;

	vec3 attn = exp(-(MieScatteringCoef * (iOdMie + Miescaleheight) + RayleighScatteringCoef * (iOdRlh + Rayleighscaleheight))) * atmosphericDensity * (1.0 - clamp(iHeight, 0.0, 1.0));
	totalRlh += odStepRlh * attn * (1.0 - density);
	totalMie += odStepMie * attn * (1.0 - density);
}

vec4 sampleAllAtmospherics(
	vec3 worldPos,
	vec3 rayDirection,
	float linear_depth,
	float highestDensityDistance,
	float density,
	float stepDistance,
	float stepCount,
	float atmosphericDensity,
	vec3 sunDirection,
	vec3 sunlightColor,
	vec3 ambientLight)
	{
	vec3 totalRlh = vec3(0,0,0);
    vec3 totalMie = vec3(0,0,0);
	float iOdRlh = 0.0;
    float iOdMie = 0.0;
	// float odStepRlh = 0.0;
	// float odStepMie = 0.0;

	const float atmosphericHeight = 40000.0;
	const vec3 RayleighScatteringCoef = vec3(5.5e-6, 13.0e-6, 22.4e-6);
	const float Rayleighscaleheight = 8e3;
	const float MieScatteringCoef = 21e-6;
	const float Miescaleheight = 1.2e3;
	const float MieprefferedDirection = 0.758;

	// Calculate the Rayleigh and Mie phases.
    float mu = dot(rayDirection, sunDirection);
    float mumu = mu * mu;
    float gg = MieprefferedDirection * MieprefferedDirection;
    float pRlh = 3.0 / (16.0 * PI) * (1.0 + mumu);
    float pMie = 3.0 / (8.0 * PI) * ((1.0 - gg) * (mumu + 1.0)) / (pow(1.0 + gg - 2.0 * mu * MieprefferedDirection, 1.5) * (2.0 + gg));

	vec3 curPos = vec3(0.0);
	float traveledDistance = 0.0;
	//bool sampledDistanceAtmo = false;
	float currentWeight = 0.0;

	for (float i = 0.0; i < stepCount; i++) {
		traveledDistance = stepDistance * (i + 1);

		currentWeight = density * (1.0 - clamp((highestDensityDistance - traveledDistance) / stepDistance, 0.0, 1.0));

		if (traveledDistance > linear_depth || currentWeight >= 1.0){
			traveledDistance = traveledDistance - stepDistance;
			currentWeight = 1.0 - clamp((linear_depth - traveledDistance) / stepDistance, 0.0, 1.0);
			sampleAtmospherics(curPos, atmosphericHeight, stepDistance, Rayleighscaleheight, Miescaleheight, RayleighScatteringCoef, MieScatteringCoef, atmosphericDensity, currentWeight, totalRlh, totalMie, iOdRlh, iOdMie);
			break;
		}


		curPos = worldPos + rayDirection * traveledDistance;

		sampleAtmospherics(curPos, atmosphericHeight, stepDistance, Rayleighscaleheight, Miescaleheight, RayleighScatteringCoef, MieScatteringCoef, atmosphericDensity, currentWeight, totalRlh, totalMie, iOdRlh, iOdMie);
	}

	// pRlh *= (1.0 - lightingWeight);
	// pMie *= (1.0 - lightingWeight);

	float AtmosphericsDistancePower = length(vec3(RayleighScatteringCoef * totalRlh + MieScatteringCoef * totalMie));
	vec3 atmospherics = 22.0 * (ambientLight * RayleighScatteringCoef * totalRlh + pMie * MieScatteringCoef * sunlightColor * totalMie);
	return vec4(atmospherics, AtmosphericsDistancePower);
}


void main() {
    ivec2 uv = ivec2(gl_GlobalInvocationID.xy);
	vec4 color;
	ivec2 lowres_size = ivec2(params.input_size);

	int resolutionScale = int(params.resolutionscale);
	ivec2 size = lowres_size * resolutionScale;

	vec2 depthUV = (vec2(uv) + vec2(1.0, 0.0)) / vec2(size);
	depthUV = clamp(depthUV, vec2(0.0), vec2(1.0));
	float depth = texture(depth_image, depthUV).r;
	vec4 view = inverse(genericData.proj) * vec4(depthUV*2.0-1.0,depth,1.0);
	view.xyz /= view.w;
	float linear_depth = length(view); //used to calculate depth based on the view angle, idk just works.

	vec2 clipUV = vec2(depthUV.x, depthUV.y);
	vec2 ndc = clipUV * 2.0 - 1.0;
	// Convert NDC to view space coordinates
	vec4 clipPos = vec4(ndc, 0.0, 1.0);
	vec4 viewPos = inverse(genericData.proj) * clipPos;
	viewPos.xyz /= viewPos.w;

	vec3 rd_world = normalize(viewPos.xyz);
	rd_world = mat3(genericData.view) * rd_world;
	// Define the ray properties

	vec3 raydirection = normalize(rd_world);
	vec3 rayOrigin = genericData.view[3].xyz; //center of camera for the ray origin, not worried about the screen width playing in, as it's for clouds.


	vec2 tempuv = vec2(uv);
	vec2 accumUV = vec2(tempuv.x / float(size.x), tempuv.y / float(size.y));
	accumUV = clamp(accumUV, vec2(0.0), vec2(1.0));
	if (false) {

		vec2 lowres_sizefloat = vec2(lowres_size);
		vec4 currentAccumilation = vec4(0.0);
		vec4 currentColorData = vec4(0.0);
		if (resolutionScale != 1){
			currentAccumilation = texture2D_bicubic(input_color_image, accumUV, lowres_sizefloat);
			currentColorData = texture2D_bicubic(input_data_image, accumUV, lowres_sizefloat);
		}
		else{
			currentAccumilation = texture(input_color_image, accumUV);
			currentColorData = texture(input_data_image, accumUV);
		}


		float minstep = genericData.min_step_distance;
		float maxstep = genericData.max_step_distance;

		float blurPower = genericData.blurPower;
		float maxTheoreticalStep = genericData.max_step_count * maxstep;

		blurPower = mix(blurPower, 0.0, currentColorData.b / maxTheoreticalStep);

		if (blurPower > 0.0){
			float blurHorizontal = blurPower / float(size.x);
			float blurVertical = blurPower / float(size.y);
			float blurQuality = genericData.blurQuality;
			//currentColorData = radialBlurData(currentColorData, linear_depth, input_data_image, accumUV, blurQuality * 4.0, blurVertical, blurHorizontal, blurQuality);
			currentAccumilation = radialBlurColor(currentAccumilation, input_color_image, input_data_image, accumUV, lowres_sizefloat, currentColorData.g, blurQuality * 4.0, blurVertical, blurHorizontal, blurQuality);

		}


		float density = clamp(currentAccumilation.a, 0.0, 1.0);
		float sampledDepth = currentColorData.r;
		float traveledDistance = currentColorData.g;
		float firstTraveledDistance = currentColorData.b;

		float lerp = 0.0;
		bool debugCollisions = false;
		if (traveledDistance > linear_depth){

			if (firstTraveledDistance > linear_depth){
				lerp = clamp(remap(firstTraveledDistance - linear_depth, 0.0, 1.0, 0.0, 1.0), 0.0, 1.0);
				density *= 1.0 - lerp;
			}
			else{
				//debugCollisions = true;
				lerp = clamp(remap(linear_depth - firstTraveledDistance, 0.0, minstep, 0.0, 1.0), 0.0, 1.0);
				density *= lerp;
				//density = 0.0;
			}
			// float lerp = clamp(remap(linear_depth, firstTraveledDistance, traveledDistance, 0.0, 1.0), 0.0, 1.0);
			// density *= lerp;
			// if (firstTraveledDistance < linear_depth){

			// 	density = 0.0;
			// }
			// else{
			// 	lerp = clamp(remap(firstTraveledDistance - linear_depth, minstep, maxstep, 0.0, 1.0), 0.0, 1.0);
			// 	density *= 1.0 - lerp;
			// }
			// traveledDistance = linear_depth;
		}
		density *= smoothstep(minstep, maxstep, linear_depth);
		float groundLinearFade = mix(smoothstep(maxTheoreticalStep, maxTheoreticalStep, linear_depth), 1.0, genericData.fogEffectGround);
		// vec4 color = imageLoad(color_image, uv);

		color = imageLoad(color_image, uv);
		// background sky
		if (true) {
			vec3 ambientfogdistancecolor = genericData.ambientfogdistancecolor.rgb * genericData.ambientfogdistancecolor.a;
			float atmosphericDensity = genericData.atmospheric_density;
			float directionalLightCount = genericData.directionalLightsCount;
			if (directionalLightCount > 0.0){
				for (float i = 0.0; i < directionalLightCount; i++){
					DirectionalLight light = directionalLights[int(i)];
					vec3 sundir = light.direction.xyz;
					//sampleColor = sundir;
					float sunUpWeight = smoothstep(0.0, 0.4, dot(sundir, vec3(0.0, 1.0, 0.0)));
					float lightPower = light.color.a * sunUpWeight;
					if (lightPower > 0.0){
						vec4 atmosphericData = sampleAllAtmospherics(rayOrigin, raydirection, linear_depth, traveledDistance, 0.0, linear_depth / 10.0, 10.0, atmosphericDensity, sundir, light.color.rgb * lightPower, ambientfogdistancecolor);
						color.rgb = mix(color.rgb, atmosphericData.rgb, atmosphericData.a * groundLinearFade); //causes jitter in the sky
					}
				}
			}
		}
		color.rgb = mix(color.rgb, currentAccumilation.rgb, density);
	}
	else {
		color = texture(input_color_image, accumUV);
		// color = imageLoad(color_image, uv);
	}


	// if (debugCollisions){
	// 	color.rgb = vec3(lerp, 0.0, 0.0);
	// }

    imageStore(color_image, uv, color);
	// if (resolutionScale != 1){
	// 	imageStore(reflections_sample, ivec2(accumUV * vec2(lowres_size)), vec4(color.rgb, traveledDistance));
	// }
	// else{
	// 	imageStore(reflections_sample, uv, vec4(color.rgb, traveledDistance));
	// }
}