#ifndef NOVA_PERIPHERAL_DEF_H
#define NOVA_PERIPHERAL_DEF_H

#define UBLOX_CUSTOM_MAX_WAIT (250u)
#define SPI_SPEED_SD_MHZ (20)

#define DELAY(MS) vTaskDelay(pdMS_TO_TICKS(MS))

#define SEALEVELPRESSURE_HPA (1013.25)


//Kalman filters
template<typename T>
union vec3_u
{
  struct
  {
    T values[3]{};
  };

  struct
  {
    T x;
    T y;
    T z;
  };

  template <typename... Args>
  explicit vec3_u(Args &&...args) : x{T(std::forward<Args>(args)...)},
                                    y{T(std::forward<Args>(args)...)},
                                    z{T(std::forward<Args>(args)...)} {}
};

#endif