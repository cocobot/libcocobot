#include <cocobot/encoders.h>
#include <platform.h>

static int32_t _enc_value[2];
static uint16_t _enc_last_angle[2];

static void cocobot_encoders_update(int id)
{
  uint16_t raw;

  if(id == 0)
  {
    platform_spi_position_select(PLATFORM_SPI_ENCR_SELECT);
  }
  else
  {
    platform_spi_position_select(PLATFORM_SPI_ENCL_SELECT);
  }

  raw = platform_spi_position_transfert(0xff) & 0xFF;
  raw |= (platform_spi_position_transfert(0xff) << 8) & 0xFF00;

  platform_spi_position_select(PLATFORM_SPI_CS_UNSELECT);

  //swap octets + delete MSB
  raw = ((raw & 0xff) << 8) | (raw >> 8);
  raw <<= 1;

  //compute new angle by abusing 16 bits integer overflow
  int16_t delta = raw - _enc_last_angle[id];
  _enc_last_angle[id] = raw;
  if(id == 0)
  {
    _enc_value[id] += delta;
  }
  else
  {
    _enc_value[id] -= delta;
  }
}

void cocobot_encoders_get_motor_position(int32_t motor_position[2])
{
  cocobot_encoders_update(0);
  cocobot_encoders_update(1);
  motor_position[0] = _enc_value[0];
  motor_position[1] = _enc_value[1];
}
