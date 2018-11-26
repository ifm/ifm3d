# Confidence bits of the O3D explained

The values in the confidence matrix contain information about the status of the corresponding ToF pixel. A confidence value has 8 bits.

|Bit| Name | Meaning when bit is set | Contributes to Pixel Validity |
|---|------|-------------------------|-------------------------------|
|0 | **Pixel is invalid** | To determine if a pixel is valid or not, only this bit needs to be checked. The distance values, as well as the Cartesian coordinates of invalid pixels are set to 0. The reason for invalidity is encoded in some of the other bits below. | yes |
|1| **Pixel is saturated** | Pixel amplitude is saturated. | yes |
|2| **Bad A-B symmetry** | The A-B symmetry vale of the four phase measurement is above threshold. Remark: This symmetry value is used to detect motion artifacts. Noise (e.g. due to strong ambient light or very short integration times) or PMD interference may also contribute. | yes |
|3| **Low amplitude** | The amplitude value is below minimum amplitude threshold. | yes |
|[5, 4]| **Exposure time indicator** | The two bits indicate which exposure time was used in a multiple exposure measurement. 00 = unused. 01 = shortest exposure time (only used in 3 exposure mode). 10 = middle exposure time in 3 exposure mode, short exposure in double exposure mode. 11 = longest exposure time (always 1 in single exposure mode) | no |
|6| **Clipped Pixel** | Clipping box on 3D data. If clipping is active this bit indicates that the pixel coordinates are outside the defined volume. | yes |
|7| **Suspect/defective Pixel** | This pixel has been marked as "suspect" or "defective" and values have been replaced by interpolated values from the surroundings. | no |
