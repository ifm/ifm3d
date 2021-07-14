- [Description of the O3X Imager Parameters](#description-of-the-o3x-imager-parameters)
  - [Basic Imager Settings](#basic-imager-settings)
    - [Type](#type)
    - [ExposureTime](#exposuretime)
    - [ExposureTimeRatio](#exposuretimeratio)
    - [FrameRate](#framerate)
    - [MaxAllowedFrameRate](#maxallowedframerate)
    - [CompensateAmbientLightDrift](#compensateambientlightdrift)
    - [MinimumAmplitude](#minimumamplitude)
    - [SymmetryThreshold](#symmetrythreshold)
  - [Filtering](#filtering)
    - [SpatialFilter.MaskSize](#spatialfiltermasksize)
    - [SpatialFilterType](#spatialfiltertype)
    - [TemporalFilterType](#temporalfiltertype)
    - [TemporalFilter](#temporalfilter)
  - [Automatic Exposure](#automatic-exposure)
    - [ContinuousAutoExposure](#continuousautoexposure)
    - [AutoExposureMaxExposureTime](#autoexposuremaxexposuretime)
    - [AutoExposureMaxNumberOfSaturatedPixels](#autoexposuremaxnumberofsaturatedpixels)
    - [AutoExposureReferencePointX AutoExposureReferencePointY](#autoexposurereferencepointx-autoexposurereferencepointy)
    - [AutoExposureReferenceROI](#autoexposurereferenceroi)
    - [AutoExposureReferenceType](#autoexposurereferencetype)
    - [AutoExposureTimeLowerLimit](#autoexposuretimelowerlimit)
  - [Straylight Correction](#straylight-correction)
    - [EnableStraylightCorrection](#enablestraylightcorrection)
    - [AbsDistStraylightThreshold](#absdiststraylightthreshold)
    - [RelAmpStraylightThreshold](#relampstraylightthreshold)
  - [Noise Estimation](#noise-estimation)
    - [EnableNoiseEstimation](#enablenoiseestimation)
    - [DistNoiseThreshold](#distnoisethreshold)

# Description of the O3X Imager Parameters
This document describes the available settings for the O3X "Imager" section of the parameter JSON.
Please note that all parameters might not be available on your device, because they were introduced in 
a more recent firmware version.

## Basic Imager Settings

### Type 
Define the exposure mode and unambiguous range.  
The first part of the string (up to the underscore) specifies the maximum unique range. Ranges $\lt$ 7m use only one 
modulation frequency - all others use 2 frequencies.  
The second part of the string defines the exposure mode. 'low' stands for low dynamic range scenarios and uses one 
exposure time, whereas 'moderate' uses double exposure for measuring objects with lower and higher reflectivity at 
once.

Default: upTo30m_moderate  
Possible values are: 
- upTo02m_low
- upTo02m_moderate
- upTo03m_low
- upTo03m_moderate
- upTo07m_low
- upTo07m_moderate
- upTo15m_low
- upTo15m_moderate
- upTo30m_low
- upTo30m_moderate


### ExposureTime
Exposure time in microseconds. In case of [double exposure](#type), this value applies to the longer exposure time.

Default: 1000 µs  
Value Range: depending on the exposure mode

### ExposureTimeRatio
In case of [double exposure](#type), the [exposure time](#exposuretime) divided by this value defines the shorter 
exposure time.

Default: 10   
Value Range: 10..50

### FrameRate
Desired frame rate in frames per second.

Default: 5 FPS  
Value Range: 0.0167..30.0 FPS

### MaxAllowedFrameRate
The maximum allowed frame rate for the current application parameters. The free-run [frame rate](#framerate) is limited 
to this value. In software triggered mode triggers will be blocked accordingly.

This parameter is read-only.

### CompensateAmbientLightDrift
Enable compensation of distance errors caused by ambient light.  

Default: false

### MinimumAmplitude
Pixels with a raw amplitude below this threshold will be invalidated.

Default: 42  
Value Range: 0..10000

### SymmetryThreshold
Threshold for the maximum allowed raw phase symmetry. Using lower values will lead to less valid pixels.

Default: 0.4  
Value Range: 0..1000

## Filtering

### SpatialFilter.MaskSize
For spatial filtering, currently only mask size 0 (equals a 3x3 filter mask) is supported.

### SpatialFilterType
Selects a filter that is applied to amplitude and distance images.
- 0 No Filtering
- 1 Median Filtering

### TemporalFilterType
Select a filter that is applied pixelwise to consecutive amplitude and distance images. Possible values are: 
- 0 No Filtering 
- 1 Adaptive Exponential Filtering: A weighted average across successive images is calculated. More recent images have 
more weight than older images. The filter can only be used in continuous trigger mode ("TriggerMode": "1").

### TemporalFilter
This item is currently unused.


## Automatic Exposure

### ContinuousAutoExposure
Enable continuous auto exposure.

Default: false

### AutoExposureMaxExposureTime
Maximum exposure time for auto exposure in microseconds.

Default: 10000 µs  
Value Range: 10..inf µs

### AutoExposureMaxNumberOfSaturatedPixels
Maximum number of allowed saturated pixels for auto exposure. Only when this threshold is 
exceeded, auto-exposure will try to find a better exposure time for the scene.

Default: 20 pixels  
Value Range: 10..10000 pixels

### AutoExposureReferencePointX AutoExposureReferencePointY
Currently not supported.

### AutoExposureReferenceROI
JSON description of the Region of Interest on which auto exposure is applied. The default 
setting defines a rectangle, which covers the whole image.

### AutoExposureReferenceType
Currently not supported.  

### AutoExposureTimeLowerLimit
Minimum exposure time for auto exposure in microseconds. For single exposure, the auto 
exposure algorithm ensures that the exposure time is greater or equal to this setting. For double exposure, the auto 
exposure algorithm ensures that the long exposure time is greater or equal to this setting.

Default: 0 µs  
Value Range: 0..1000 µs

## Straylight Correction

### EnableStraylightCorrection
Enable straylight correction.

Default: false

### AbsDistStraylightThreshold
Absolute distance threshold in meters for invalidating pixels due to straylight correction. 
Using lower values might lead to less valid pixels, depending on the amount of straylight in the scene. Hence, the 
remaining valid pixels are less affected by straylight.  
Setting a threshold of 0.0m disables the check, so that no pixels will be invalidated due to this criterion.  

Default: 0.05 m  
Value Range: 0..1 m

### RelAmpStraylightThreshold
Relative amplitude threshold for invalidating pixels due to straylight correction. 
Using lower values might lead to less valid pixels, depending on the amount of straylight in the scene. Hence, the 
remaining valid pixels are less affected by straylight.  
Setting a threshold of 0 disables the check, so that no pixels will be invalidated due to this criterion.

Default: 0.2  
Value Range: 0..1

## Noise Estimation

### EnableNoiseEstimation
Enable distance noise estimation. When set to true, the standard deviation in meters of the distance noise is estimated 
by sensor noise considerations. Note that image filters are not incorporated in the noise estimation, hence the noise 
observed in the temporally or spatially filtered distance will be lower than this estimation.

Default: false

### DistNoiseThreshold
Only when [noise estimation](#enablenoiseestimation) is activated: Maximum allowed distance noise in meters 
(0.0: threshold is disabled).

Default: 0.0 m (no threshold)  
Value Range: 0..100 m
