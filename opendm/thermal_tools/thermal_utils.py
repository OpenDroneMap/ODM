"""Thermal Image manipulation utilities."""
"""Based on https://github.com/detecttechnologies/thermal_base"""
import numpy as np

def sensor_vals_to_temp(
    raw,
    Emissivity=1.0,
    ObjectDistance=1,
    AtmosphericTemperature=20,
    ReflectedApparentTemperature=20,
    IRWindowTemperature=20,
    IRWindowTransmission=1,
    RelativeHumidity=50,
    PlanckR1=21106.77,
    PlanckB=1501,
    PlanckF=1,
    PlanckO=-7340,
    PlanckR2=0.012545258,
    **kwargs,):
    """Convert raw values from the thermographic sensor sensor to temperatures in °C. Tested for Flir and DJI cams."""
    # this calculation has been ported to python from https://github.com/gtatters/Thermimage/blob/master/R/raw2temp.R
    # a detailed explanation of what is going on here can be found there

    # constants
    ATA1 = 0.006569
    ATA2 = 0.01262
    ATB1 = -0.002276
    ATB2 = -0.00667
    ATX = 1.9

    # transmission through window (calibrated)
    emiss_wind = 1 - IRWindowTransmission
    refl_wind = 0

    # transmission through the air
    h2o = (RelativeHumidity / 100) * np.exp(
        1.5587
        + 0.06939 * (AtmosphericTemperature)
        - 0.00027816 * (AtmosphericTemperature) ** 2
        + 0.00000068455 * (AtmosphericTemperature) ** 3
    )
    tau1 = ATX * np.exp(-np.sqrt(ObjectDistance / 2) * (ATA1 + ATB1 * np.sqrt(h2o))) + (1 - ATX) * np.exp(
        -np.sqrt(ObjectDistance / 2) * (ATA2 + ATB2 * np.sqrt(h2o))
    )
    tau2 = ATX * np.exp(-np.sqrt(ObjectDistance / 2) * (ATA1 + ATB1 * np.sqrt(h2o))) + (1 - ATX) * np.exp(
        -np.sqrt(ObjectDistance / 2) * (ATA2 + ATB2 * np.sqrt(h2o))
    )
    # radiance from the environment
    raw_refl1 = PlanckR1 / (PlanckR2 * (np.exp(PlanckB / (ReflectedApparentTemperature + 273.15)) - PlanckF)) - PlanckO
    
    # Reflected component
    raw_refl1_attn = (1 - Emissivity) / Emissivity * raw_refl1  
    
    # Emission from atmosphere 1
    raw_atm1 = (
        PlanckR1 / (PlanckR2 * (np.exp(PlanckB / (AtmosphericTemperature + 273.15)) - PlanckF)) - PlanckO
    )  
    
    # attenuation for atmospheric 1 emission
    raw_atm1_attn = (1 - tau1) / Emissivity / tau1 * raw_atm1  
    
    # Emission from window due to its own temp
    raw_wind = (
        PlanckR1 / (PlanckR2 * (np.exp(PlanckB / (IRWindowTemperature + 273.15)) - PlanckF)) - PlanckO
    )  
    # Componen due to window emissivity
    raw_wind_attn = (
        emiss_wind / Emissivity / tau1 / IRWindowTransmission * raw_wind
    )  
    # Reflection from window due to external objects
    raw_refl2 = (
        PlanckR1 / (PlanckR2 * (np.exp(PlanckB / (ReflectedApparentTemperature + 273.15)) - PlanckF)) - PlanckO
    )  
    # component due to window reflectivity
    raw_refl2_attn = (
        refl_wind / Emissivity / tau1 / IRWindowTransmission * raw_refl2
    )  
    # Emission from atmosphere 2
    raw_atm2 = (
        PlanckR1 / (PlanckR2 * (np.exp(PlanckB / (AtmosphericTemperature + 273.15)) - PlanckF)) - PlanckO
    )  
    # attenuation for atmospheric 2 emission
    raw_atm2_attn = (
        (1 - tau2) / Emissivity / tau1 / IRWindowTransmission / tau2 * raw_atm2
    )

    raw_obj = (
        raw / Emissivity / tau1 / IRWindowTransmission / tau2
        - raw_atm1_attn
        - raw_atm2_attn
        - raw_wind_attn
        - raw_refl1_attn
        - raw_refl2_attn
    )
    val_to_log = PlanckR1 / (PlanckR2 * (raw_obj + PlanckO)) + PlanckF
    if any(val_to_log.ravel() < 0):
        raise Exception("Image seems to be corrupted")
    # temperature from radiance
    return PlanckB / np.log(val_to_log) - 273.15


def parse_from_exif_str(temp_str):
    """String to float parser."""
    # we assume degrees celsius for temperature, metres for length
    if isinstance(temp_str, str):
        return float(temp_str.split()[0])
    return float(temp_str)


def normalize_temp_matrix(thermal_np):
    """Normalize a temperature matrix to the 0-255 uint8 image range."""
    num = thermal_np - np.amin(thermal_np)
    den = np.amax(thermal_np) - np.amin(thermal_np)
    thermal_np = num / den
    return thermal_np

def clip_temp_to_roi(thermal_np, thermal_roi_values):
    """
    Given an RoI within a temperature matrix, this function clips the temperature values in the entire thermal.

    Image temperature values above and below the max/min temperatures within the RoI are clipped to said max/min.

    Args:
        thermal_np (np.ndarray): Floating point array containing the temperature matrix.
        thermal_roi_values (np.ndarray / list): Any iterable containing the temperature values within the RoI.

    Returns:
        np.ndarray: The clipped temperature matrix.
    """
    maximum = np.amax(thermal_roi_values)
    minimum = np.amin(thermal_roi_values)
    thermal_np[thermal_np > maximum] = maximum
    thermal_np[thermal_np < minimum] = minimum
    return thermal_np


def scale_with_roi(thermal_np, thermal_roi_values):
    """Alias for clip_temp_to_roi, to be deprecated in the future."""
    return clip_temp_to_roi(thermal_np, thermal_roi_values)