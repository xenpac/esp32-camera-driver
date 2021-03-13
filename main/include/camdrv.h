//camdrv.h
#include "sensor.h"

int ov2640_init(sensor_t *sensor);
int camera_init(void);
sensor_t *get_camstruct(void);

int i2sInit(int fbsize);
int init_framebuffer(int framesize);
void I2S_Start();
void I2S_Stop();
struct framebuffer *i2s_getframe(void);
void i2s_unlockfb(void);

int SCCB_Init(int pin_sda, int pin_scl);
uint8_t SCCB_Probe(void);
uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg);
uint8_t SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data);
uint8_t SCCB_Read16(uint8_t slv_addr, uint16_t reg);
uint8_t SCCB_Write16(uint8_t slv_addr, uint16_t reg, uint8_t data);


struct framebuffer
{
    uint8_t *pbuf; // pointer to the FrameBuffer data
    uint32_t len; // number of valid bytes in this framebuffer
};
