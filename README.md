<img src="https://github.com/Bonkahe/SunshineClouds2/blob/main/githubstuff/Logo.png?raw=true">

# SunshineClouds

## Updates:
version 2.0:
Rebuild from the ground up to use the compositor system, improving performance and visuals.

## Features
* Fully volumetric clouds, extendable as far from the camera as you want with the correct settings
* Easy Setup (besides getting the addon installed it comes down to like 3 clicks)
* Performant (I use resolution options with upscaling and depth handling, allow for soft clouds at low resolution you can still fly through)
* Soft visuals, or cartoony, styling options allows for a wide range of options
* Entirely procedural, or paint in clouds using the editor tools, able to paint in density as well as color to tint clouds differently depending on region

## Installation

1. Download either here or in the asset library by searching for "Sunshine Clouds 2"
2. Activate Plugin (Project->Project Settings->Plugins->SunshineClouds2)
3. Add a SunshineCloudsDriverGD node to your scene
4. Ensure your scene has a World Environment present in it
5. Press "Generate Clouds Resource" button in the driver
6. Add a directional light to the "Tracked Directional Lights" array on the driver
7. Have fun!

## Contribution
Feel free to make pr's or issues here, I will fix them as able, I will love to see whatever you all come up with!

### Current outstanding issues
* Need to implement screen space shadows
* When shadow cookies are available (A pr is currently live with someone working on this in the Godot repo, I will pursue this whenever they get it merged)


## License
This plugin has been released under the [MIT License](https://github.com/Bonkahe/SunshineClouds2/blob/main/LICENSE).
