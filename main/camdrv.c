/* Camera Driver
This file contains:
- the Camera driver. all stuff related a the camera module (not the i2s!)
- basicly, powerup, supply camera clock, init I2C,  init camera and supply camera-struct to the application.
- we hold the camera struct (to be compatible with espressif camera software)
- sccb driver
- resolution structure defining framesizes

we are using sensor.h file to be compatible with esp implementations of camera drivers.

 Camera Module: OV2640

march 2021, Thomas Krueger
*/

#include "driver/ledc.h"
#include "esp_log.h"
#include "camdrv.h"
#include "driver/i2c.h"


//interface pins:
#define I2CDATAPIN 26
#define I2CCLOCKPIN 27
#define XCLKPIN 0
#define POWERPIN 32  // use -1 if not used

//camera initial settings:
#define FREQUENCY 20000000
#define FRAMESIZE FRAMESIZE_VGA
#define PIXFORMAT PIXFORMAT_JPEG
#define JPGQUALITY 12   //this is the default ov2640. lesser values may cause imagedata corruption from the camera!
//i2s settings
#define BUFFERSIZE 380000   // max jpeg framebuffer size. make a good guess ;)

static const char *TAG = "CAMDRV";



//protos:
int camera_init(void);
sensor_t *get_camstruct(void);
esp_err_t start_clock(gpio_num_t pin, int xclk_freq_hz);
esp_err_t set_freq( int xclk_freq_hz);

//global data:
sensor_t sensor; // camera functions and state

// returns 0 on success, 1 on error
int camera_init(void)
{
    uint8_t c;
    int ret=0;

    SCCB_Init(I2CDATAPIN, I2CCLOCKPIN);
    start_clock(XCLKPIN,FREQUENCY); // start camera xclk on pin 0 at 20mhz. results in pclk=5Mhz in jpg.

//power on camera with power cycle(reset)
    if (POWERPIN != -1)
    {
        gpio_set_direction(POWERPIN, GPIO_MODE_OUTPUT);

        gpio_set_level(POWERPIN, 1); //off
        vTaskDelay(200 / portTICK_PERIOD_MS);
        gpio_set_level(POWERPIN, 0); //on
        vTaskDelay(200 / portTICK_PERIOD_MS);// turn on
    }

    ESP_LOGI(TAG, "probing cam address");
    c=SCCB_Probe();
    ESP_LOGI(TAG, "Got:0x%02x",c);

    sensor.slv_addr=c;

    switch (c)
    {
    case 0:
        ESP_LOGE(TAG, "NO camera detected!");
        return 1;
    case 0x30:
        ESP_LOGI(TAG, "OV2640 detected");
        ov2640_init(&sensor); // get the cam function pointers
        break;

    default:
        ESP_LOGE(TAG, "UNKNOWN camera detected at Adr:0x%02x",c);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        return 1;
    }
    //cam init
    ret+=sensor.reset(&sensor); //important, it writes the register list!
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ret+=sensor.set_pixformat(&sensor, PIXFORMAT);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ret+=sensor.set_framesize(&sensor, FRAMESIZE);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ret+=sensor.set_gainceiling(&sensor, GAINCEILING_2X);
    ret+=sensor.set_bpc(&sensor, false);
    ret+=sensor.set_wpc(&sensor, true);
    ret+=sensor.set_quality(&sensor, JPGQUALITY);
    ret+=sensor.init_status(&sensor); // get current camera status into sensor struct
    vTaskDelay(100 / portTICK_PERIOD_MS);

    if (ret)
        ESP_LOGE(TAG, "camera init failed!!");
    else
        ESP_LOGI(TAG, "camera init OK");

    ret=i2sInit(BUFFERSIZE); // init capture driver

    return ret;
}

// we are holding the camera struct for the application
sensor_t *get_camstruct(void)
{
    return (&sensor);
}

// can be called anytime to change frequency. upto 40Mhz in 5Mhz steps, because duty is set to 1.
esp_err_t set_freq( int xclk_freq_hz)
{
    int ledc_timer=0; // use timer 0
    ledc_timer_config_t timer_conf;
    timer_conf.duty_resolution = 1; //was 2
    timer_conf.freq_hz = xclk_freq_hz;
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
#if ESP_IDF_VERSION_MAJOR >= 4
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
#endif
    timer_conf.timer_num = (ledc_timer_t)ledc_timer;
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK)
    {
        printf("ledc_timer_config failed for freq %d, rc=%x", xclk_freq_hz, err);
    }
    return err;
}


// initial ledc timer init
esp_err_t start_clock(gpio_num_t pin, int xclk_freq_hz)
{
    periph_module_enable(PERIPH_LEDC_MODULE);

    esp_err_t err = set_freq( xclk_freq_hz);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG,"ledc_timer_config failed, rc=%x", err);
        return err;
    }


    ledc_channel_config_t ch_conf;
    ch_conf.gpio_num = pin; // output pin
    ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    ch_conf.channel = 0;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.timer_sel = 0;
    ch_conf.duty = 1; //was 2
    ch_conf.hpoint = 0;
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG,"ledc_channel_config failed, rc=%x", err);
        return err;
    }
    ESP_LOGI(TAG,"Clock attach OK, %d Mhz",xclk_freq_hz/1000000);
    return ESP_OK;
}



// sccb stuff:


int SCCB_Init(int pin_sda, int pin_scl)
{
    ESP_LOGI(TAG, "pin_sda %d pin_scl %d\n", pin_sda, pin_scl);
    ESP_LOGI(TAG, "HW I2c");
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = pin_sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = pin_scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 200000;   // i2c bus clock!!
    conf.clk_flags = 0;
    i2c_param_config(1, &conf);
    i2c_driver_install(1, conf.mode, 0, 0, 0);

    return 0;
}

uint8_t SCCB_Probe()
{
    uint8_t slave_addr = 0;

    while(slave_addr < 0x7f)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( slave_addr << 1 ) | 0, 1);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(1, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if( ret == ESP_OK)
        {
            return slave_addr; // got response = valid slv_addr
        }
        slave_addr++;
    }

    if (slave_addr==0x7F) slave_addr=0; // indicate not found
    return slave_addr;
}

uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg)
{
    uint8_t data=0;
    esp_err_t ret = ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slv_addr << 1 ) | 0, 1);
    i2c_master_write_byte(cmd, reg, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(1, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "sccb Read failed");
        return -1;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slv_addr << 1 ) | 1, 1);
    i2c_master_read_byte(cmd, &data, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(1, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SCCB_Read Failed addr:0x%02x, reg:0x%02x, data:0x%02x, ret:%d", slv_addr, reg, data, ret);
    }
    return data;
}

uint8_t SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data)
{

    esp_err_t ret = ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slv_addr << 1 ) | 0, 1);
    i2c_master_write_byte(cmd, reg, 1);
    i2c_master_write_byte(cmd, data, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(1, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SCCB_Write Failed addr:0x%02x, reg:0x%02x, data:0x%02x, ret:%d", slv_addr, reg, data, ret);
    }
    return ret == ESP_OK ? 0 : -1;

}


const resolution_info_t resolution[FRAMESIZE_INVALID] =
{
    {   96,   96, ASPECT_RATIO_1X1   }, /* 96x96 */
    {  160,  120, ASPECT_RATIO_4X3   }, /* QQVGA */
    {  176,  145, ASPECT_RATIO_5X4   }, /* QCIF adapted!! */
    {  240,  176, ASPECT_RATIO_4X3   }, /* HQVGA */
    {  240,  240, ASPECT_RATIO_1X1   }, /* 240x240 */
    {  320,  240, ASPECT_RATIO_4X3   }, /* QVGA  */
    {  352,  290, ASPECT_RATIO_4X3   }, /* CIF  adapted!! */
    {  480,  320, ASPECT_RATIO_3X2   }, /* HVGA  */
    {  640,  480, ASPECT_RATIO_4X3   }, /* VGA   */
    {  800,  600, ASPECT_RATIO_4X3   }, /* SVGA  */
    { 1024,  768, ASPECT_RATIO_4X3   }, /* XGA   */
    { 1280,  720, ASPECT_RATIO_16X9  }, /* HD    */
    { 1280, 1024, ASPECT_RATIO_5X4   }, /* SXGA  */
    { 1600, 1200, ASPECT_RATIO_4X3   }, /* UXGA  */
    // 3MP Sensors
    { 1920, 1080, ASPECT_RATIO_16X9  }, /* FHD   */
    {  720, 1280, ASPECT_RATIO_9X16  }, /* Portrait HD   */
    {  864, 1536, ASPECT_RATIO_9X16  }, /* Portrait 3MP   */
    { 2048, 1536, ASPECT_RATIO_4X3   }, /* QXGA  */
    // 5MP Sensors
    { 2560, 1440, ASPECT_RATIO_16X9  }, /* QHD    */
    { 2560, 1600, ASPECT_RATIO_16X10 }, /* WQXGA  */
    { 1088, 1920, ASPECT_RATIO_9X16  }, /* Portrait FHD   */
    { 2560, 1920, ASPECT_RATIO_4X3   }, /* QSXGA  */
};
