#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>

#define UBLOX_CUSTOM_MAX_WAIT (250u)

struct Data
{
    uint32_t timestamp;

    double gps_latitude;
    double gps_longitude;
    float gps_altitude;
};
Data data;
uint16_t sat;

// i2c devices
TwoWire i2c3(PB8, PA8);
SFE_UBLOX_GNSS m10q;

void setup()
{
    Serial.begin(460800);
    delay(2000);

    i2c3.begin();

    pinMode(PA0, OUTPUT);

    // m10q (0x42)
    if (m10q.begin(0x42, UBLOX_CUSTOM_MAX_WAIT))
    {
        m10q.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
        m10q.setNavigationFrequency(5, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
        m10q.setAutoPVT(true, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
        m10q.setDynamicModel(DYN_MODEL_AUTOMOTIVE, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
        Serial.println("success");
    }
}

void loop()
{
    if (m10q.getPVT())
    {
        data.timestamp = m10q.getUnixEpoch(UBLOX_CUSTOM_MAX_WAIT);
        sat = m10q.getSIV(UBLOX_CUSTOM_MAX_WAIT);
        data.gps_latitude = static_cast<double>(m10q.getLatitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        data.gps_longitude = static_cast<double>(m10q.getLongitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        data.gps_altitude = static_cast<float>(m10q.getAltitudeMSL(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3f;
    }

    Serial.print("GPS Latitude: ");
    Serial.println(data.gps_latitude, 6);
    Serial.print("GPS Longitude: ");
    Serial.println(data.gps_longitude, 6);
    Serial.print("GPS Altitude: ");
    Serial.println(data.gps_altitude);
    Serial.print("Satellite: ");
    Serial.println(sat);
    delay(1000);
}