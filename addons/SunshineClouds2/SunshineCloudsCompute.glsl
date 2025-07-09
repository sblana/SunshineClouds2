#[compute]
#version 450
#define PI 3.141592
#define ABSORPTION_COEFFICIENT 0.9

// Invocations in the (x, y, z) dimension
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

layout(rgba16f, binding = 0) uniform image2D output_data_image;
layout(rgba16f, binding = 1) uniform image2D output_color_image;

layout(rgba32f, binding = 2) uniform image2D accum_1A_image;
layout(rgba32f, binding = 3) uniform image2D accum_1B_image;

layout(rgba32f, binding = 4) uniform image2D accum_2A_image;
layout(rgba32f, binding = 5) uniform image2D accum_2B_image;

layout(binding = 6) uniform sampler2D depth_image;
layout(binding = 7) uniform sampler2D extra_large_noise;
layout(binding = 8) uniform sampler3D large_noise;
layout(binding = 9) uniform sampler3D noise_medium;
layout(binding = 10) uniform sampler3D noise_small;
layout(binding = 11) uniform sampler3D curl_noise;
layout(binding = 12) uniform sampler3D dither_small;
layout(binding = 13) uniform sampler2D heightmask;

layout(binding = 14) uniform uniformBuffer {
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
	float powderStrength;
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
	float windSweptRange;
	float windSweptPower;
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

layout(binding = 15) uniform LightsBuffer {
	DirectionalLight directionalLights[4];
	PointLight pointLights[128];
	PointEffector pointEffectors[64];
};

layout(binding = 16, std430) restrict buffer SamplePointsBuffer {
	vec4 SamplePoints[32];
};

// Our push constant
layout(push_constant, std430) uniform Params {
	vec2 raster_size;
	float large_noise_scale;
	float medium_noise_scale;

	float time;
	float cloud_coverage;
	float cloud_density;
	float small_noise_strength;

	float cloud_lighting_power;
	float accumilation_decay;
	vec2 cameraRotation;
} params;

//Helpers
const int BayerFilter16[16] =
{
    0, 8, 2, 10,
    12, 4, 14, 6,
    3, 11, 1, 9,
    15, 7, 13, 5
};
const int BayerFilter4[4] =
{
    0, 1,
    3, 2,
};

const mat4 bayer_matrix = mat4(
    vec4(00.0 / 16.0, 12.0 / 16.0, 03.0 / 16.0, 15.0 / 16.0),
    vec4(08.0 / 16.0, 04.0 / 16.0, 11.0 / 16.0, 07.0 / 16.0),
    vec4(02.0 / 16.0, 14.0 / 16.0, 01.0 / 16.0, 13.0 / 16.0),
    vec4(10.0 / 16.0, 06.0 / 16.0, 09.0 / 16.0, 05.0 / 16.0));

float quadraticOut(float t) {
  return -t * (t - 2.0);
}

float quadraticIn(float t) {
  return t * t;
}

float rand(vec2 co){
    return fract(sin(dot(co, vec2(12.9898, 78.233))) * 43758.5453);
}

float get_dither_value(vec2 pixel) {
    int x = int(pixel.x - 4.0 * floor(pixel.x / 4.0));
    int y = int(pixel.y - 4.0 * floor(pixel.y / 4.0));
    return bayer_matrix[x][y];
}

float remap(float value, float min1, float max1, float min2, float max2) {
  return min2 + (value - min1) * (max2 - min2) / (max1 - min1);
}

float BeersLaw (float dist, float absorption) {
  return exp(-dist * absorption);
}

float Powder (float dist, float absorption) {
  return 1.0 - exp(-dist * absorption * 2.0);
}

float HenyeyGreenstein(float g, float costh)
{
    return (1.0 - g * g) / (4.0 * PI * pow(1.0 + g * g - 2.0 * g * costh, 3.0/2.0));
}

bool renderBayer(ivec2 fragCoord, int framecount)
{
	//int BAYER = 16;
    //int index = framecount % BAYER;
    
    return (fragCoord.x + 4 * fragCoord.y) % 16 == BayerFilter16[framecount];
}

//Sample functions

float sampleScene(
	vec3 largeNoisePos, 
	vec3 mediumNoisePos, 
	vec3 smallNoisePos, 
	vec3 worldPosition, 
	float cloudceiling, 
	float cloudfloor, 
	float extralargeNoiseValue,
	float largenoisescale, 
	float mediumnoisescale, 
	float smallnoisescale, 
	float coverage, 
	float smallscalePower, 
	float curlPower, 
	float lod, 
	bool ambientsample)
	{
	float clampedWorldHeight = remap(worldPosition.y, cloudfloor, cloudceiling, 0.0, 1.0);
	vec4 gradientSample = texture(heightmask, vec2(clampedWorldHeight, 0.5)).rgba;
	

	float edgeFade = min(smoothstep(0.0, 0.1, clampedWorldHeight), smoothstep(1.0, 0.9, clampedWorldHeight));
	float extraLargeShape = extralargeNoiseValue * gradientSample.b;

	float smallShape = texture(noise_small, (worldPosition - smallNoisePos) / smallnoisescale).r;

	float curlHeightSample = (1.0 - gradientSample.a);

	float effectorAdditive = 0.0;
	vec2 WindDirection = genericData.WindDirection;
	worldPosition += vec3(WindDirection.x, 0.0, WindDirection.y) * genericData.windSweptPower * quadraticIn(1.0 - clamp(clampedWorldHeight / genericData.windSweptRange, 0.0, 1.0));

	if (lod > 0.0){
		for (int i = 0; i < int(genericData.pointEffectorCount); i++){
			float effectorDistance = distance(pointEffectors[i].position, worldPosition);
			if (effectorDistance < pointEffectors[i].radius){
				effectorAdditive += mix(pointEffectors[i].power, 0.0, effectorDistance / pointEffectors[i].radius) * edgeFade;
			}
		}

		if (!ambientsample && curlHeightSample > 0.0 && min(curlPower, lod) > 0.5){
			
			float curlLod = remap(lod, 0.5, 1.0, 0.0, 1.0);
			worldPosition += (((texture(curl_noise, (worldPosition - mediumNoisePos) / mediumnoisescale).xyz * 2.0) - 1.0) * vec3(1.0, 0.2, 1.0) + vec3(WindDirection.x, 0.0, WindDirection.y) * 0.9) * curlPower * curlHeightSample * curlLod;
			worldPosition += (((texture(curl_noise, (worldPosition - mediumNoisePos) / mediumnoisescale).xyz * 2.0) - 1.0) * vec3(1.0, 0.2, 1.0) + vec3(WindDirection.x, 0.0, WindDirection.y) * 0.9) * curlPower * curlHeightSample * curlLod;
			worldPosition += (((texture(curl_noise, (worldPosition - mediumNoisePos) / mediumnoisescale).xyz * 2.0) - 1.0) * vec3(1.0, 0.2, 1.0) + vec3(WindDirection.x, 0.0, WindDirection.y) * 0.9) * curlPower * curlHeightSample * curlLod;
			
			clampedWorldHeight = remap(worldPosition.y, cloudfloor, cloudceiling, 0.0, 1.0);
			gradientSample = texture(heightmask, vec2(clampedWorldHeight, 0.5)).rgba;
		}
	}

	float largeShape = texture(large_noise, (worldPosition - largeNoisePos) / largenoisescale).r * extraLargeShape;
	largeShape = smoothstep(coverage , coverage - 0.1, 1.0 - (largeShape * gradientSample.r)) + max(effectorAdditive, 0.0);
	vec4 mediumShapes = texture(noise_medium, (worldPosition - mediumNoisePos) / mediumnoisescale).rgba;
	float mediumshape = 1.0 - mediumShapes.b;
	smallShape = smallShape * gradientSample.g * pow((1.0 - mediumshape), smallscalePower);
	

	float shape = mediumshape + max(effectorAdditive, 0.0);
	shape = clamp(remap(shape, 1.0 - largeShape, 1.0, 0.0, 1.0), 0.0, 1.0);
	shape = clamp(remap(shape, smallShape, 1.0, 0.0, 1.0), 0.0, 1.0);
	shape += min(effectorAdditive, 0.0);

	return clamp((shape * edgeFade), 0.0, 1.0);
}

float sampleLighting(
	int stepCount, 
	vec3 worldPosition,
	vec3 extralargeNoisePos, 
	vec3 largeNoisePos, 
	vec3 mediumNoisePos, 
	vec3 smallNoisePos, 
	vec3 sunDirection,
	float densityMultiplier,
	float sunUpWeight, 
	float stepDistance,  
	float cloudceiling, 
	float cloudfloor, 
	float extralargenoisescale,
	float largenoisescale, 
	float mediumnoisescale, 
	float smallnoisescale, 
	float coverage, 
	float smallscalePower, 
	float curlPower, 
	float lod)
	{
	float density = 0.0;
	float stepCountFloat = max(float(stepCount) * lod, 2.0);
	float actualDistance = mix(stepDistance * 4.0, stepDistance, lod);
	float eachShortStep = actualDistance / (float(stepCount) / stepCountFloat) / stepCountFloat;
	float traveledDistance = 0.0;
	
	float sunUpValue = 1.0 - sunUpWeight;
	float eachStepWeight = 1.0 / stepCountFloat;

	float heightGradient = 0.0;
	float thisDensity = 0.0;
	float count = 0.0;
	vec3 curPos = worldPosition;
	for (float i = 0.0; i < stepCountFloat; i++) {
		traveledDistance = mix(eachShortStep, actualDistance, clamp(quadraticOut(i / stepCountFloat), 0.0, 1.0));
		curPos = worldPosition + sunDirection * traveledDistance;

		if (density < 1.0 && clamp(curPos.y, cloudfloor, cloudceiling) == curPos.y){
			heightGradient = remap(curPos.y, cloudfloor, cloudceiling, 0.0, 1.0);
			
			heightGradient = clamp(smoothstep(sunUpValue - 0.1, sunUpValue, heightGradient), 0.0, 1.0);
			float extraLargeShape = texture(extra_large_noise, (curPos.xz - extralargeNoisePos.xz) / extralargenoisescale).a;

			thisDensity = sampleScene(largeNoisePos, mediumNoisePos, smallNoisePos, curPos, cloudceiling, cloudfloor, extraLargeShape, largenoisescale, mediumnoisescale, smallnoisescale, coverage, smallscalePower, curlPower, lod, true) * densityMultiplier * eachStepWeight;
			density += mix(1.0, thisDensity, heightGradient);
		}
		else{
			break;
		}
	}

	return density;
}

float sampleAO(
	vec3 extralargeNoisePos,
	vec3 largeNoisePos, 
	vec3 mediumNoisePos, 
	vec3 smallNoisePos, 
	vec3 worldPosition, 
	float lightingSampleRange, 
	float cloudceiling, 
	float cloudfloor,
	float extralargenoisescale,
	float largenoisescale, 
	float mediumnoisescale, 
	float smallnoisescale, 
	float coverage, 
	float smallscalePower, 
	float curlPower, 
	float lod)
	{
	vec3 samplePos = worldPosition;
	samplePos.y += lightingSampleRange * 0.5;
	samplePos.y += lightingSampleRange * (rand(samplePos.xz) * 2.0 - 1.0);
	samplePos.x += lightingSampleRange * (rand(samplePos.zy) * 2.0 - 1.0);
	samplePos.z += lightingSampleRange * (rand(samplePos.yx) * 2.0 - 1.0);

	float extraLargeShape = texture(extra_large_noise, (samplePos.xz - extralargeNoisePos.xz) / extralargenoisescale).a;
	return sampleScene(largeNoisePos, mediumNoisePos, smallNoisePos, samplePos, cloudceiling, cloudfloor, extraLargeShape, largenoisescale, mediumnoisescale, smallnoisescale, coverage, smallscalePower, curlPower, lod, true);
}

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


void main() {
	//SETTING UP UVS/RAY DATA
	ivec2 uv = ivec2(gl_GlobalInvocationID.xy);
	ivec2 size = ivec2(params.raster_size);

	// Prevent reading/writing out of bounds.
	if (uv.x >= size.x || uv.y >= size.y) {
		return;
	}
	
	vec2 depthUV = vec2(float(uv.x) / float(size.x), float(uv.y) / float(size.y));
	float depth = texture(depth_image, depthUV).r;
	vec4 view = inverse(genericData.proj) * vec4(depthUV*2.0-1.0,depth,1.0);
	view.xyz /= view.w;
	float linear_depth = length(view); //used to calculate depth based on the view angle, idk just works.

	
	// Convert screen coordinates to normalized device coordinates
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


	//DITHER
	float ditherScale = 40.037;
	vec3 ditherUV = vec3(depthUV.x * ditherScale , depthUV.y * ditherScale , params.time);
	float smallNoise = texture(dither_small, ditherUV).r;

	float ditherValue = smallNoise;

	//ATMOSPHERICS
	vec3 ambientfogdistancecolor = genericData.ambientfogdistancecolor.rgb;
	vec3 totalRlh = vec3(0,0,0);
    vec3 totalMie = vec3(0,0,0);
	float iOdRlh = 0.0;
    float iOdMie = 0.0;
	float atmosphericDensity = genericData.atmospheric_density;

	const float atmosphericHeight = 40000.0;
	const vec3 RayleighScatteringCoef = vec3(5.5e-6, 13.0e-6, 22.4e-6);
	const float Rayleighscaleheight = 8e3;
	const float MieScatteringCoef = 21e-6;
	const float Miescaleheight = 1.2e3;
	const float MieprefferedDirection = 0.758;

	//IMPORTED DATA
	int stepCount = int(genericData.max_step_count);
	int lightingStepCount = int(genericData.max_lighting_step_count);
	int directionalLightCount = int(genericData.directionalLightsCount);
	int pointLightCount = int(genericData.pointLightsCount);

	vec3 extralargeNoisePos = genericData.extralargenoiseposition;
	vec3 largeNoisePos = genericData.largenoiseposition;
	vec3 mediumNoisePos = genericData.mediumnoiseposition;
	vec3 smallNoisePos = genericData.smallnoiseposition;

	float extralargenoiseScale = genericData.extralargenoisescale;
	float largenoiseScale = params.large_noise_scale;
	float mediumnoiseScale = params.medium_noise_scale;
	float smallnoiseScale = genericData.small_noise_scale;

	float minstep = genericData.min_step_distance;
	float maxstep = genericData.max_step_distance;

	float curlPower = genericData.curlPower;
	float lightingStepDistance = genericData.lighting_step_distance;
	float cloudfloor = genericData.cloud_floor;
	float cloudceiling = genericData.cloud_ceiling;

	float densityMultiplier = params.cloud_density;
	float sharpness = clamp(1.0 - genericData.cloud_sharpness, 0.001, 1.0) * 2.0;
	float lightingSharpness = genericData.cloud_lighting_sharpness;
	float smallNoiseMultiplier = params.small_noise_strength;

	float coverage = params.cloud_coverage * 1.01;
	float lightingdensityMultiplier = params.cloud_lighting_power;
	lightingdensityMultiplier += lightingdensityMultiplier * 3.0 * coverage;

	vec4 aobase = genericData.ambientGroundLightColor;
	
	//bool debugCollisions = false;
	//int frameIndex = int(genericData.filterIndex);
	
	//REUSABLE VARIABLES
	bool override = false;
	bool densityBreak = false;
	bool depthBreak = false;

	float maxTheoreticalStep = float(stepCount) * maxstep;
	float highestDensity = 0.0;
	float highestDensityDistance = maxTheoreticalStep;
	float ceilingSample = cloudceiling;
	float lodMaxDistance = maxstep * float(stepCount) * genericData.lod_bias;
	float halfcloudThickness = (cloudceiling - cloudfloor) * 0.5;
	float halfCeiling = cloudceiling - halfcloudThickness;
	

	float newStep = maxstep * ditherValue;
	float traveledDistance = newStep;

	vec4 currentColorAccumilation = vec4(0.0);
	vec4 currentDataAccumilation = vec4(0.0);




	//Used for interlaced rendering, not currently enabled due to it's long accumilation time, results in a lot of noticable artifacts.
	//Though it does improve performance, so maybe for some people it will be helpful.

				//bool rebuildFrame = renderBayer(uv, frameIndex);
				// bool rebuildFrame = true;
				
				// if (!rebuildFrame){
				// 	//accumulation preperation:
				// 	vec4 niaveDataRetreval = vec4(0.0);
				// 	float usingaccumA = params.cameraRotation.x;
				// 	if (usingaccumA > 0.0){
				// 		niaveDataRetreval = imageLoad(accum_2A_image, uv).rgba;
				// 	}
				// 	else{
				// 		niaveDataRetreval = imageLoad(accum_2B_image, uv).rgba;
				// 	}
				// 	//depthBreak = niaveDataRetreval.r > linear_depth;

				// 	vec3 worldFinalPos = curPos + raydirection * niaveDataRetreval.g;
				// 	worldFinalPos += (rayOrigin - genericData.prevview[3].xyz);
				// 	//Prevview is already actually the inv_view (due to the way retrieving the transform works), so inversing it here is making it the equalivant of View_Matrix.
				// 	vec4 reprojectedClipPos = inverse(genericData.prevview) * vec4(worldFinalPos, 1.0);
					
					
				// 	if (reprojectedClipPos.z > 0.0){
				// 		override = true;
				// 	}
				// 	else{
				// 		vec4 reprojectedScreenPos = genericData.prevproj * reprojectedClipPos;
						
				// 		// Convert clip space to normalized device coordinates
				// 		ndc = (reprojectedScreenPos.xy / reprojectedScreenPos.w);

				// 		// Convert normalized device coordinates to screen space
				// 		vec2 screen_position = ndc * 0.5 + 0.5;
				// 		//screen_position = clamp(screen_position, vec2(0.0), vec2(1.0));
				// 		screen_position = screen_position - depthUV;
				// 		ivec2 adjustedUV = ivec2(int(screen_position.x * size.x), int(screen_position.y * size.y));
				// 		//float change = length(vec2(adjustedUV));
				// 		adjustedUV += uv; //Size is the screen resolution.
						
				// 		ivec2 clampedUV = clamp(adjustedUV, ivec2(0), size - ivec2(1)); //having two lets me check if clamping it changed the reprojected uv, if it did that means it was offscreen, so rebuild data.

				// 		//execute accumilation.
				// 		float accumdecay = params.accumilation_decay;

				// 		//alternate back and forth to avoid stepping on pixels being written too.
						
				// 		float actualDepth = abs(reprojectedClipPos.z);
						
				// 		if (usingaccumA > 0.0){
				// 			currentDataAccumilation = imageLoad(accum_2A_image, adjustedUV).rgba;
				// 			bool lastDepthBreak = currentDataAccumilation.a < 0.0;
				// 			float sampledDepth = currentDataAccumilation.r;
				// 			depthBreak = actualDepth > sampledDepth;
				// 			if (clampedUV != adjustedUV || depthBreak != lastDepthBreak){
				// 				override = true;
				// 				//debugCollisions = true;
				// 			}
				// 			else{
				// 				imageStore(accum_1B_image, uv, imageLoad(accum_1A_image, adjustedUV));
				// 				imageStore(accum_2B_image, uv, currentDataAccumilation);
				// 			}
							
				// 		}
				// 		else{
				// 			currentDataAccumilation = imageLoad(accum_2B_image, adjustedUV).rgba;
				// 			bool lastDepthBreak = currentDataAccumilation.a < 0.0;
				// 			float sampledDepth = abs(currentDataAccumilation.r);
				// 			depthBreak = actualDepth > sampledDepth;
				// 			if (clampedUV != adjustedUV || depthBreak != lastDepthBreak){
				// 				override = true;
				// 				//debugCollisions = true;
				// 			}
				// 			else{
				// 				imageStore(accum_1A_image, uv, imageLoad(accum_1B_image, adjustedUV));
				// 				imageStore(accum_2A_image, uv, currentDataAccumilation);

				// 			}
				// 		}
				// 	}

				// }
				
	// END INTERLACED RENDERING


	
	//if (rebuildFrame || override){ //Re-enable for interlaced rendering
	//If it is our render, build the data for this pixel
	
	
	vec3 directionalLightSunUpPower[4] = vec3[4](vec3(0.0), vec3(0.0), vec3(0.0), vec3(0.0));
	float totalLightPower = 0.0;

	for (int lightI = 0; lightI < directionalLightCount; lightI++){
		if (directionalLights[lightI].color.a > 0.0){
			
			directionalLightSunUpPower[lightI].r = smoothstep(-0.03, 0.07, dot(directionalLights[lightI].direction.xyz, vec3(0.0, 1.0, 0.0)));
			totalLightPower += directionalLights[lightI].color.a * directionalLightSunUpPower[lightI].r;

			directionalLightSunUpPower[lightI].b = dot(directionalLights[lightI].direction.xyz, raydirection);
		}
	}
	

	
	
	vec4 lightColor = vec4(0.0);
	vec3 paintedColor = vec3(0.0);
	float initialdistanceSample = -1.0;

	float lightingSamples = 0.0;

	float density = 0.0;
	float ambient = 0.0;
	float depthFade = 1.0;
	float newdensity = 0.0;
	vec3 curPos = vec3(0.0);
	
	float curLod = 1.0;
	float samplePosCount = genericData.samplePointsCount;

	if (samplePosCount > 0 && uv == ivec2(0)){
		for (int i = 0; i < samplePosCount; i++){
			curPos = SamplePoints[i].xyz;
			vec4 maskSample = texture(extra_large_noise, (curPos.xz - extralargeNoisePos.xz) / extralargenoiseScale);
			ceilingSample = mix(halfCeiling, cloudceiling, maskSample.a);
			
			SamplePoints[i].w = pow(sampleScene(largeNoisePos, mediumNoisePos, smallNoisePos, curPos, ceilingSample, cloudfloor, maskSample.a, largenoiseScale, mediumnoiseScale, smallnoiseScale, coverage, smallNoiseMultiplier, curlPower, 1.0, false) * densityMultiplier, sharpness);
		}
	}

	for (int i = 0; i < stepCount; i++) {
		
		if (traveledDistance > linear_depth){
			depthFade = 1.0 - smoothstep(linear_depth - newStep, linear_depth, traveledDistance);
			depthBreak = true;
		}
		
		curPos = rayOrigin + raydirection * traveledDistance;
		
		vec4 maskSample = texture(extra_large_noise, (curPos.xz - extralargeNoisePos.xz) / extralargenoiseScale);
		ceilingSample = mix(halfCeiling, cloudceiling, maskSample.a);
		
		sampleAtmospherics(curPos, atmosphericHeight, newStep, Rayleighscaleheight, Miescaleheight, RayleighScatteringCoef, MieScatteringCoef, atmosphericDensity, density, totalRlh, totalMie, iOdRlh, iOdMie); 
			
		if (clamp(curPos.y, cloudfloor, cloudceiling) == curPos.y){

			curLod = 1.0 - clamp(traveledDistance / lodMaxDistance, 0.0, 1.0);
			newdensity = pow(sampleScene(largeNoisePos, mediumNoisePos, smallNoisePos, curPos, ceilingSample, cloudfloor, maskSample.a, largenoiseScale, mediumnoiseScale, smallnoiseScale, coverage, smallNoiseMultiplier, curlPower, curLod, false) * densityMultiplier, sharpness) * depthFade;
			
			
			
			if (newdensity > 0.0){
				if (initialdistanceSample < 0.0){
					initialdistanceSample = traveledDistance;
				}

				float powderEffect = pow(newdensity, genericData.powderStrength * 2.0);

				paintedColor += maskSample.rgb;
				lightingSamples += 1.0;
				for (int lightI = 0; lightI < directionalLightCount; lightI++){
					if (directionalLights[lightI].color.a > 0.0){
						
						vec3 sundir = directionalLights[lightI].direction.xyz;
						float sunUpWeight = directionalLightSunUpPower[lightI].r;

						int thislightingStepCount = min(int(directionalLights[lightI].direction.w), lightingStepCount);
						if (thislightingStepCount > 0){
							float henyeygreenstein =  pow(HenyeyGreenstein(genericData.anisotropy, directionalLightSunUpPower[lightI].b), mix(1.0, 2.0, 1.0 - genericData.anisotropy)); 
							float densitySample = sampleLighting(thislightingStepCount, curPos, extralargeNoisePos, largeNoisePos, mediumNoisePos, smallNoisePos, sundir, densityMultiplier * lightingdensityMultiplier, sunUpWeight, lightingStepDistance, ceilingSample, cloudfloor, extralargenoiseScale, largenoiseScale, mediumnoiseScale, smallnoiseScale, coverage, smallNoiseMultiplier, curlPower, curLod);
							densitySample = BeersLaw(lightingStepDistance, densitySample * henyeygreenstein);
							//densitySample = Powder(lightingStepDistance, densitySample);
							float thisStepLightingWeight = (clamp(pow(densitySample, lightingSharpness), 0.0, 1.0)) * sunUpWeight;
							

							lightColor.rgb += pow(directionalLights[lightI].color.rgb * directionalLights[lightI].color.a * thisStepLightingWeight, vec3(2.2)) * powderEffect;
							directionalLightSunUpPower[lightI].g += directionalLights[lightI].color.a * thisStepLightingWeight;
						}
						else{
							lightColor.rgb += pow(directionalLights[lightI].color.rgb * directionalLights[lightI].color.a * sunUpWeight, vec3(2.2)) * powderEffect;
							directionalLightSunUpPower[lightI].g += directionalLights[lightI].color.a * sunUpWeight;
						}

						
					}
				}

				for (int lightI = 0; lightI < pointLightCount; lightI++){
					vec3 lightToOriginDelta = pointLights[lightI].position.xyz - curPos;
					float lightDistanceWeight = length(lightToOriginDelta); 
					if (pointLights[lightI].color.a > 0.0 && lightDistanceWeight < pointLights[lightI].position.w){
						lightToOriginDelta = normalize(lightToOriginDelta);
						//float densitySample = 1.0 - newdensity;
						float densitySample = sampleLighting(3, curPos, extralargeNoisePos, largeNoisePos, mediumNoisePos, smallNoisePos, lightToOriginDelta, densityMultiplier, 1.0, min(maxstep, lightDistanceWeight), ceilingSample, cloudfloor, extralargenoiseScale, largenoiseScale, mediumnoiseScale, smallnoiseScale, coverage, smallNoiseMultiplier, curlPower, curLod);
						
						float henyeygreenstein = pow(HenyeyGreenstein(genericData.anisotropy, dot(lightToOriginDelta, raydirection)), mix(1.0, 2.0, 1.0 - genericData.anisotropy)); 
						densitySample = BeersLaw(lightDistanceWeight, densitySample * henyeygreenstein);
						densitySample = mix(densitySample, newdensity, 0.5) * powderEffect;
						lightDistanceWeight = lightDistanceWeight / pointLights[lightI].position.w;
						lightDistanceWeight = pointLights[lightI].color.a * pow((1.0 - lightDistanceWeight), 2.2) * densitySample;


						lightColor.rgb += pow(pointLights[lightI].color.rgb * lightDistanceWeight, vec3(2.2));
					}
				}
				
				if (aobase.a > 0.0){
					ambient += sampleScene(largeNoisePos, mediumNoisePos, smallNoisePos, curPos + vec3(0.0, 1.0, 0.0) * minstep, ceilingSample, cloudfloor, maskSample.a, largenoiseScale, mediumnoiseScale, smallnoiseScale, coverage, smallNoiseMultiplier, curlPower, curLod, true) * densityMultiplier * lightingdensityMultiplier ;
				}

				
				newStep = mix(mix(maxstep, minstep, pow(newdensity, 0.1)), maxstep, float(i) / float(stepCount));
				if (newdensity > highestDensity){
					highestDensity = newdensity;
					highestDensityDistance = traveledDistance;
				}
			}
			else{
				newStep = maxstep;
			}

			if (i == 0){
				newdensity = mix(newdensity, 0.0, clamp(traveledDistance / maxstep, 0.0, 1.0));
			}

			density += newdensity;
			if (density >= 1.0){
				densityBreak = true;
				break;
			}
		}
		else{
			if (min(curPos.y - cloudceiling, raydirection.y) > 0.0 || max(curPos.y - cloudfloor, raydirection.y) < 0.0){
				
				traveledDistance = min(maxTheoreticalStep, linear_depth);
				curPos = rayOrigin + raydirection * traveledDistance;
				
				//debugCollisions = true;
				break;
			}
			
			newStep = maxstep;
		}
		

		if (depthBreak){
			break;
		}
		traveledDistance += newStep;
	}


	ambient = clamp(ambient / lightingSamples, 0.0, 1.0);
	paintedColor = clamp(paintedColor / lightingSamples, 0.0, 1.0);


	vec3 ambientLight = genericData.ambientLightColor.rgb * clamp(totalLightPower, 0.0, 1.0);
	ambientLight = mix(ambientLight, ambientLight * aobase.rgb, clamp(ambient * aobase.a, 0.0, 1.0)) * paintedColor;
	lightColor.rgb = ambientLight + clamp(lightColor.rgb / lightingSamples, vec3(0.0), vec3(2.0));
	lightColor.a = density;


	for (int lightI = 0; lightI < directionalLightCount; lightI++){
		if (directionalLights[lightI].color.a > 0.0){
			float sunUpWeight = clamp(directionalLightSunUpPower[lightI].r / lightingSamples, 0.0, 1.0);
			float sunAOPower = clamp(directionalLightSunUpPower[lightI].g / lightingSamples, 0.0, 1.0);
			float mu = dot(raydirection, directionalLights[lightI].direction.xyz);
			

			float mumu = mu * mu;
			float gg = MieprefferedDirection * MieprefferedDirection;
			float pRlh = 3.0 / (16.0 * PI) * (1.0 + mumu);
			float pMie = 3.0 / (8.0 * PI) * ((1.0 - gg) * (mumu + 1.0)) / (pow(1.0 + gg - 2.0 * mu * MieprefferedDirection, 1.5) * (2.0 + gg));

			float AtmosphericsDistancePower = length(vec3(RayleighScatteringCoef * totalRlh + MieScatteringCoef * totalMie));
			vec3 atmospherics = 22.0 * (ambientfogdistancecolor * RayleighScatteringCoef * totalRlh + pMie * MieScatteringCoef * (directionalLights[lightI].color.rgb * sunAOPower) * totalMie);

			lightColor.rgb = mix(lightColor.rgb, atmospherics, (AtmosphericsDistancePower * sunUpWeight)); //causes jitter in the sky
		}
	}

	initialdistanceSample = max(initialdistanceSample, 0.0);


	//accumulation preperation:
	float finalDensityDistance = min(traveledDistance, highestDensityDistance);
	vec3 worldFinalPos = rayOrigin + raydirection * (highestDensityDistance + maxstep);
	vec3 delta = rayOrigin - genericData.prevview[3].xyz;
	worldFinalPos += delta;

	//Prevview is already actually the inv_view (due to the way retrieving the transform works), so inversing it here is making it the equalivant of View_Matrix.
	vec4 reprojectedClipPos = inverse(genericData.prevview) * vec4(worldFinalPos, 1.0);
	
	reprojectedClipPos.z -= 0.01;
	if (reprojectedClipPos.z > 0.0){
		override = true;
	}
	
	vec4 reprojectedScreenPos = genericData.prevproj * reprojectedClipPos;

	// Convert clip space to normalized device coordinates
	ndc = (reprojectedScreenPos.xy / reprojectedScreenPos.w);

	// Convert normalized device coordinates to screen space
	vec2 screen_position = ndc * 0.5 + 0.5;
	//screen_position = clamp(screen_position, vec2(0.0), vec2(1.0));
	screen_position = screen_position - depthUV;

	ivec2 adjustedUV = ivec2(int(screen_position.x * size.x), int(screen_position.y * size.y));
	//float change = length(vec2(adjustedUV));
	adjustedUV += uv; //Size is the screen resolution.
	
	ivec2 clampedUV = clamp(adjustedUV, ivec2(0), size - ivec2(1)); //having two lets me check if clamping it changed the reprojected uv, if it did that means it was offscreen, so rebuild data.

	//execute accumilation.
	float accumdecay = params.accumilation_decay;

	//alternate back and forth to avoid stepping on pixels being written too.
	float usingaccumA = params.cameraRotation.x;
	
	//float finalDensityDistance = max(traveledDistance, highestDensityDistance);
	//linear_depth = max(linear_depth, traveledDistance);
	float travelspeed = length(delta) + maxstep;
	//bool debugCollisions = false;
	if (usingaccumA > 0.0){
		currentColorAccumilation = imageLoad(accum_1A_image, adjustedUV).rgba;
		currentDataAccumilation = imageLoad(accum_2A_image, adjustedUV).rgba;

		bool lastDepthBreak = currentDataAccumilation.a < 0.0;

		if (override || clampedUV != adjustedUV || (depthBreak != lastDepthBreak && abs(linear_depth - currentDataAccumilation.r) > travelspeed)){
			currentColorAccumilation = lightColor;
			//debugCollisions = true;
			currentDataAccumilation.r = linear_depth;
			currentDataAccumilation.g = finalDensityDistance;
			currentDataAccumilation.b = initialdistanceSample;
		}
		else{
			currentColorAccumilation = (currentColorAccumilation * accumdecay) + lightColor * (1.0 - accumdecay);

			currentDataAccumilation.r = mix(currentDataAccumilation.r, linear_depth,  (1.0 - accumdecay));
			currentDataAccumilation.g = mix(currentDataAccumilation.g, finalDensityDistance,  (1.0 - accumdecay));
			currentDataAccumilation.b = mix(currentDataAccumilation.b, initialdistanceSample,  (1.0 - accumdecay));
		}

		if (depthBreak){
			currentDataAccumilation.a = -1.0;
		}
		else{
			currentDataAccumilation.a = 1.0;
		}

		imageStore(accum_1B_image, uv, currentColorAccumilation);
		imageStore(accum_2B_image, uv, currentDataAccumilation);
	}
	else{
		currentColorAccumilation = imageLoad(accum_1B_image, adjustedUV).rgba;
		currentDataAccumilation = imageLoad(accum_2B_image, adjustedUV).rgba;

		bool lastDepthBreak = currentDataAccumilation.a < 0.0;

		if (override || clampedUV != adjustedUV || (depthBreak != lastDepthBreak && abs(linear_depth - currentDataAccumilation.r) > travelspeed)){
			currentColorAccumilation = lightColor;
			//debugCollisions = true;
			currentDataAccumilation.r = linear_depth;
			currentDataAccumilation.g = finalDensityDistance;
			currentDataAccumilation.b = initialdistanceSample;
		}
		else{
			currentColorAccumilation = (currentColorAccumilation * accumdecay) + lightColor * (1.0 - accumdecay);

			currentDataAccumilation.r = mix(currentDataAccumilation.r, linear_depth,  (1.0 - accumdecay));
			currentDataAccumilation.g = mix(currentDataAccumilation.g, finalDensityDistance,  (1.0 - accumdecay));
			currentDataAccumilation.b = mix(currentDataAccumilation.b, initialdistanceSample,  (1.0 - accumdecay));
		}

		if (depthBreak){
			currentDataAccumilation.a = -1.0;
		}
		else{
			currentDataAccumilation.a = 1.0;
		}

		imageStore(accum_1A_image, uv, currentColorAccumilation);
		imageStore(accum_2A_image, uv, currentDataAccumilation);
	}

	currentDataAccumilation.a = abs(currentDataAccumilation.a);
	imageStore(output_color_image, uv, currentColorAccumilation);
	imageStore(output_data_image, uv, currentDataAccumilation);
	//}
}
