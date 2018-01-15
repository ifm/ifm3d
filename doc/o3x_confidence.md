# Confidence bits of the O3X explained

The values in the confidence matrix contain information about the status of the corresponding ToF pixel. A confidence value has 8 bits.

Bit| Meaning when bit is set|
|---|-------------------------------|
|0 | **Pixel is invalid** To determine if a pixel is valid or not, only this bit needs to be checked. The distance values, as well as the Cartesian coordinates of invalid pixels are set to 0. The reason for invalidity is encoded in some of the other bits below. |
|1| **Saturation** Signal strength of the ambient and/or modulated light exceeds the upper limit.|
|2| **Implausible measurement** Four phase validation of this pixel failed due to movement during the exposure or other kind of disturbances like an interfering modulated light source in the observed scene.|
|3| **Low amplitude** Integrated signal strength of the own modulated light was not sufficient for a  reliable distance measurement.|
|4| **Long exposure time taken** In double exposure modes this bit indicates whether the long or the short integration time was selected.|
|5| **Double frequencymeasurementmismatch** In double frequency modes this bit indicates a implausible measurement due to a mismatch of the two frequency distance values. Fast movement in the scenery can be a reason for this bit to be set.|
|6| **Reserved** |
|7| **Reserved** |
