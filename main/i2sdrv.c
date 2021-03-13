/*i2s camera capture driver only for jpeg:

This is a driver for the esp32 or esp32-s processors  "I2S" interface doing 8-Bit parallel Input.
 It can be used as:
 - camera input capture for jpeg

 Interface has:
- 8 Data pins
- 1 Data strobe pin. (PCLK , WS, or similar)
- optinally VSYNC, HREF for camera can be used to gate the input data. special pin numbers for constant level are: 0x30 = 0; 0x38 = 1;

ESP32 project config:
- must use 240Mhz cpuclock
- must use 80MHZ SPIRAM clock

As jpeg on low quality uses small framebuffers, we might get away to use internal DRAM memory, which is much faster than SPIRAM, thus higher FPS.
We use 2 Framebuffers default as ping-pong buffers.

JPEG mode:
- start-Marker = 0xFFD8 at start of frame
- end-Marker   = 0xFFD9 at end of frame
- in a jpg image there is no 0xff except in the start/endmarkers.

- when ov2640 is sending jpeg encoded images, the received bytecount depends on the quality setting.
Its about 20 to 100K Bytes, no constant value.
HREF toggles for every 4 to 10 bytes received, so about 2000 to 8000 toggles per frame, unpredictable.
To capture jpeg, we use the vsync-interrupt.
during vsync-high, we collect all received data into framebuffer.(ignoring linelength)(HREF gates the incoming valid data, so we shure get valid data)
When vsync goes low, there is the situation, that the last inlink descriptor might not get finished, leaving some data in the last dmabuffer without interrupt.
Therefore an additional inlink transaction is manually triggered from the lastused descriptor to have all data in the framebuffer.(+ some optional garbage)
The jpeg endmarker FFD9 is then searched for to have the exact frame ending.
This happens in the VSYNC interrupt.
Then, for the next frame,  the i2s/DMA engine is completely reset to have proper start for the next frame.

MaxSampleSpeed: (PCLK)
- 7mhz works in all samplemodes with data-read to SPIRAM framebuffer!
- 12mhz works in all samplemodes with data-read to internal DRAM framebuffer (fastest)
- 14 mhz works in all samplemodes WITHOUT data-read! so dataread slows down.

feb-2021 Thomas Krueger, Hofgeismar Germany  (all rights reserved)
*/

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/periph_ctrl.h"
#include "esp_intr_alloc.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp32/rom/lldesc.h"
#include "esp_heap_caps.h"
#include "esp32/spiram.h"
#include "esp32/himem.h"
#include "camdrv.h"



static const char *TAG = "I2S";



//+++protos:

void I2S_Start(void);
void I2S_Stop(void);
void i2sConfReset();

int i2sInit(int fbsize);
int init_framebuffer(int framesize);


//+++global data:

// +++PIN definitions of the parallel I2s interface:
// use 0x30 for constant LOW, or 0x38 for constant HIGH Levels, as PIN-number, if there is no related real IO-signal.
//esp32-cam PIN Map for OV2640:
#define D7      35
#define D6      34
#define D5      39
#define D4      36
#define D3      21
#define D2      19
#define D1      18
#define D0      5
#define VSYNC   25
#define HREF    23
#define PCLK    22 // this is the data strobe signal, rising edge (falling edge also possible to config)

uint8_t PinTab[]= {VSYNC, HREF, PCLK, D0, D1, D2, D3, D4, D5, D6, D7};

// +++DMA buffer stuff:
/*
The DMA buffer descriptor is a hardware specific structure used by the esp32 DMA controller.
   DMA Desc struct, aka lldesc_t

  --------------------------------------------------------------
  | own | EoF | sub_sof | 5'b0   | length [11:0] | size [11:0] |
  --------------------------------------------------------------
  |            buf_ptr [31:0]                                  |
  --------------------------------------------------------------
  |            next_desc_ptr [31:0]                            |
  --------------------------------------------------------------

typedef struct lldesc_s
{
    volatile uint32_t size  :12,   // total size in Bytes of the pointed-to DmaBuffer. max 4095 bytes. 12bit=4095, but 4092 as divby4
	NOTE: The size-value (DMAbuffersize) triggers the in_done INT after clocks=(size/4)*bytespersample   !!!!!!!
    length:12, // (IO-DataLen).TX=set by software = valid NuMBytes in DataBufer. RX= set by hardware= NumBytes received in DataBuffer. max 4092 bytes
    offset: 5, // h/w reserved 5bit field, s/w use it as offset in buffer.part of reserved Bitfield[6]. ?? not used
    sosf  : 1, // start of sub-frame. part of reserved Bitfield[6]. ?? not used.the reserved Bitfield[6] can be used as RW register by software.
    eof   : 1, // if set, indicates last used descriptor. set by hardware in RX. set by software in TX. (for eof_int)
    owner : 1; // hw or sw . should be 1 for DMA. cleared by hardware at end of current DMA transaction.is not evaluated by hardware!

	HARDWARE clears "owner" and sets "length" to received bytes in that "buf" in all participating Descriptors.
	          It sets "eof" in the last participating Descriptor and clears "eof" in all preceeding Descriptors.
			  Hardware does not evaluate "owner"!
			  Sooo no need to update these settings prior to a transaction.!

    volatile uint8_t *buf;       // pointer to memory address of related DataBuffer, used by dma as destination
    union{
        volatile uint32_t empty;
        STAILQ_ENTRY(lldesc_s) qe;  // DRAM pointer to next descriptor, or NULL if last descriptor in the chain. used by dma
    };
} lldesc_t; // DMA buffer descriptor struct, as defined in esp-idf/components/esp32/include/rom/lldesc.h

The DMA controller will start transfers using the first descriptor which is given to him by the I2S module.
When the related dmabuffer is full(in_done is triggered), it will automaticly load the next descriptor from the linked list until
the transfer count is exhausted(if it was given). It will then set the eof-bit in that last descriptor(eof_int is triggered if enabled).
It will then start over with the first descriptor again......etc.
DMA transfers are triggered by the I2S-Fifo logic which is always the source/destination to/from memory(DRAM prefered!) for the DMA.
The minimum transfersize is 4Bytes or 1 DWORD(32bit).
*/


// DMA stuff:
#define DMABUFSIZE 1024  //4092 max, must be multible of 4 and fit in 12 bits!
#define NDESCRIPTORS 4  // max descriptors to use. 
DMA_ATTR lldesc_t DMAdesc[NDESCRIPTORS];
DMA_ATTR uint8_t DMAbuf[NDESCRIPTORS*DMABUFSIZE]; // we use a single buffer of size NDESCRIPTORS * DMABUFSIZE
int NumDes; // number of used descriptors
int CurDes; // index of current descriptor



// ringbuffer size
#define NBUFFERS 2
// reserve space for NBUFFERS Framebuffer-structures
DMA_ATTR struct framebuffer FB[NBUFFERS][sizeof(struct framebuffer)];
uint8_t *FrameBuffer;// pointer to the currently used framebuffer for I2S
int numFB=0; // number of exisiting Framebuffers
volatile int readFB; // index of current readable framebuffer
volatile int writeFB; // index currently writing to
volatile int DataCnt; // count of Bytes in current framebuffer (offset)
int FrameSize;
int FrameError;


//I2S stuff
intr_handle_t isr_i2sHandle = 0;
int BytesPerSample;
//int SampleCount = 0; // current samplecount in DWORDs: max:4092/4 or if less: transfersize/4
int I2Sstate=0; // 0=stopped; 1=start command;2=running

// extern
extern int HwFrameCnt, I2sFrameCnt,DMAerrors,JPGerrors;
int Ihref;

//+++++  IRAM functions for interrupt +++++++++++++++++++++

/*
 arg is just dummy for that intr_handler_t which defines a parameter.

 This service routine is used to read captured i2s data from the filled DMAbuffers.
 In camera non-jpeg capture this would be one complete line of the image.

• I2S_IN_SUC_EOF_INT: Triggered when all requested data has been received and the eof-flag is set in the last participating descriptor.
                      If DMAbuffer data wasnt copied by int_done ints, you may now do so for all DMAbuffers. (possibly in the line-blank period)
	NOTE: if jpg data is captured, the requested amount of data may not be received due to compression and this INT will not happen!
	      It is better to use in_done ints to collect the data then.
		  Even then, the last descriptor may not succeed/complete, trigger an in_done INT.
		  However the actual received amount of Bytes is written in the descriptors length field.


• I2S_IN_DONE_INT: Triggered when the current rxlink descriptor is handled.
				   So it happens for every participating descriptor!
                   It comes before the I2S_IN_SUC_EOF_INT for all participating descriptors.
				   So everytime the specified amount of Bytes (size) in a descriptor has been received, it will trigger this interrupt.
				   Use it to copy the amount of Bytes from the DMAbufer to the framebuffer(resultbuffer).
				   Meanwhile reception continues, so be quick else your DMAbuffer gets overwritten.(if continuous operation)


related registers:
I2S0.in_eof_des_addr =  The address of receive link descriptor producing EOF
I2S0.rx_eof_num =  The length of the data to be received in DWORDs. It will trigger I2S_IN_SUC_EOF_INT.
I2S0.in_link_dscr = address of current descriptor that triggered the in_done INT.

Interrupt service for all I2S enabled ints:
we use only in_done!
*/

void IRAM_ATTR isr_i2s(void *arg)
{
    volatile uint8_t *pdma;
    uint8_t *pdata;
    int cnt,i;

    /* the in_done int hits when a dmabuffer has been filled. and always BEFORE the suc_eof int!(if used).
    The Owner flag is cleared by DMA controller indicating Software may now use the DMAbuffer for copy.
    The oef flag is also set by hardware if the descriptor is the last in the chain.(we dont use it).
    The DMA transfered Bytecount is in length.
    Reading data here works up to 12mhz i2s clock(pclk).
    */





    I2S0.int_clr.in_done=1; // clear int flag at start to detect overruns

    /* the owner bit is cleared by DMA on descriptor complete transfer before this interrupt.
    If owner isnt cleared:
    - most likely writing/copy data is too slow, we are skipping an interrupt or part.  SPIRAM speed!! is the reason.
      DMA is using descriptor again while we still reading last entry and setting ownwer bit...then last queued interrupt hits immediately showing ownwer==1
      Solve: reduce pixelclock by lowering camera-clock.
    - wrong samplecount/linelength..check image dimensions(not for jpg)

     */
    if (!arg) // if real interrupt and not called from software
        if (DMAdesc[CurDes].owner) DMAerrors++; //overrun

    pdata=FrameBuffer;
    if (pdata) // copy only if we have a framebuffer available
    {
        pdata += DataCnt;
        // copy data from dmabuffers to framebuffer :
        pdma=DMAdesc[CurDes].buf;
        cnt=DMAdesc[CurDes].length; // in bytes
        if (BytesPerSample == 2) //(using samplemode=1)
        {
            for (i=0; i<cnt; i+=4) // copy every second byte to databuffer, second byte first
            {
                *pdata++=*(pdma+2);
                *pdata++=*pdma;
                DataCnt+=2;
                pdma+=4;
            }
        }
        else // 1 byte per sample (samplemode=3)
        {
            pdma += 2; // start at pos = 2
            for (i=0; i<cnt; i+=4) // copy every 4th byte at pos 3 to databuffer
            {
                *pdata++=*pdma;
                pdma+=4;
                DataCnt++;
            }
        }
    }
    DMAdesc[CurDes].owner=1; //give it back to DMA

    CurDes++;
    if (CurDes==NumDes) CurDes=0;

}

/* This int happens when the camera has just finished one complete frame (vertical blank period)
The blanking time (no data arrives) is quite long so we can use it to do framebuffer management.
The framebuffer data should be already there by now, except last uncompleted dma transfer.

We  also start/stop the capture sequence here as we always want to start before a camera frame is send to get all framedata.
*/

void IRAM_ATTR isr_vysnc(void *arg)
{
    int i,ok;
    uint8_t *pdata=FrameBuffer;


    if (I2Sstate==2&&pdata) // if i2s is running, and valid framebuffer pointer, process data
    {
        // get the last uncompleted dma transfer by calling the isr routine from here.
        isr_i2s((void *)1);

        if (DataCnt > FrameSize) // check if datacount is greater than framebuffer size, overflow
        {
            FrameError = DataCnt;  // for display of jpg buffer needs. we actually overrun the framebuffer now!!
            ok=-1;
        }
        else
            ok=0;

        // find the jpg startmarker
        if(*pdata == 0xFF && *(pdata+1) == 0xD8 && ok==0)
            ok=1;

        if (ok == 1)
        {
            // find the jpg endmarker
            pdata += DataCnt; // goto end of received data
            while(pdata > FrameBuffer) // go backwards
            {
                if(*pdata == 0xFF && *(pdata+1) == 0xD9)
                {
                    // if found endmarker
                    pdata += 2; //goto after 0xD9
                    DataCnt = pdata - FrameBuffer;  //adjust length
                    ok=2;
                    break;
                }
                pdata--;
            }
        }

        if (ok == 2) // if valid frame received, put to ringbuffer
        {

            I2sFrameCnt++;

            /* Ringbuffer Stuff: in interrupt here we have sole access to the ringbuffer.
             we are at writeFB and framebuffer has just been written to it.
             We always write to writeFB-framebufer until we can advance, thus having filled it already/no delay.
            ringbuffer full check. ringbuffer is full if readindex==(writeindex+1), so writeindex must stay behind readindex.
            */
            i=(writeFB+1)%numFB;
            if (i != readFB) //if not full
            {
                FB[writeFB]->len=DataCnt; // update current len
                writeFB=i; // goto next framebuffer structure
                FrameBuffer=FB[i]->pbuf; // pick its Framebuffer for next run
            }
        }
        else
            JPGerrors++;  // invalid jpg headers encountered.


        // restart the engine
        I2S0.conf.rx_start = 0; //stop i2s engine
        I2S0.in_link.stop = 1; // stop DMA engine, the inlink descriptor chain. This bit auto-clears after stop
        i2sConfReset();
        I2Sstate=1;

    } // endif state==2
    if (I2Sstate==1) // start the engine
    {

        // start the engine
        I2S0.in_link.start = 1; // start DMA engine, the inlink descriptor chain. This bit auto-clears after start!
        I2S0.conf.rx_start = 1; // start i2s engine as receiver. this bit starts(1)/stops(0) the receiver!!!

        I2Sstate=2; // running

    }

    DataCnt=CurDes=0;

    HwFrameCnt++;
//	if (gpio_get_level(VSYNC)) DMAerrors ++; //interrupt-computing took too long!! Frame already started. happens at UXGA and best quality.vsync-int doesnt like it

}

// if enabled, gives horizontal line interrupt
/*
jpeg mode:
example: bytes captured = 19200; HREF pulses per VSYNC = 2160 (variing a bit)
soo: during HREF high some Databytes are transmitted, we dont know how many...about 4 to 10 bytes
EOF=FF D9 00 00 00 00 00 00 , so no datalength info is send.
SOF=FF D8 FF E0
So..we cannot evaluate the horizontal linelength for anything useful.
we dont use this interrupt.
*/
void IRAM_ATTR isr_href(void *arg)
{
    Ihref++;
}



// Reset DMA and I2S
void i2sConfReset()
{
// reset DMA controller	, AHB interface and AHB cmdFifo
    const uint32_t lc_conf_reset_flags = I2S_IN_RST_M | I2S_AHBM_RST_M | I2S_AHBM_FIFO_RST_M;
    I2S0.lc_conf.val |= lc_conf_reset_flags; // set reset bits
    I2S0.lc_conf.val &= ~lc_conf_reset_flags; // clear reset bits

// perform fifo and receiver reset, also transmitter
    const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
    I2S0.conf.val |= conf_reset_flags;
    I2S0.conf.val &= ~conf_reset_flags;
    while (I2S0.state.rx_fifo_reset_back); // wait for reset done, this sometimes hangs!!!!
}



// +++ user callable functions ++++++++++++++++


// stop i2s engine.
void IRAM_ATTR I2S_Stop()
{
    I2S0.conf.rx_start = 0; //stop i2s engine
    I2S0.in_link.stop = 1; // stop DMA engine, the inlink descriptor chain. This bit auto-clears after stop
    I2Sstate=0;

}


/* start the i2s engine
*/
void I2S_Start()
{
    i2sConfReset(); // this is necessary. otherwise the descriptors are mixed up!

    I2S0.int_clr.val = I2S0.int_raw.val; // clear all active intflags
    //interrupts:
    I2S0.int_ena.in_done = 1; //int enable.Triggered when current inlink descriptors is handled. thats only one descriptor!!!

    I2Sstate=1;

}



/* Main Init. Init the I2S interface with given portpins set in the defines up top of file.
This shall be called only once after reset!!

uses global PinTab table and defines of portpins.
  Format: VSYNC, HREF, PCLK, D0, D1, D2, D3, D4, D5, D6, D7  for 8Bit interface
entry:
- sample mode = 1 or 3. no speed difference!
+++++ it seems that the max pclk is 10mhz (ov2640 init with 20mhz clock gives 10mhz pclk!non-jpg)
Set "Core Debug Level"=None for > 10Mhz capture operations
12mhz works in all modes including data-read!
14 mhz works in all modes without data-read! so dataread is getting too slow.
exit:
- 1 = fail
- 0 = OK, I2S is operational but not running
*/
int i2sInit(int fbsize)
{
    int i;


    BytesPerSample = 2; //samplemode=1


    ESP_LOGI(TAG,"i2sInit FBsize:%d BytesPerSample:%d",fbsize,BytesPerSample);

    gpio_config_t conf =
    {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    }; // init the port config struct to use for all pins

    for (i = 0; i < sizeof(PinTab); i++)
    {
        if (rtc_gpio_is_valid_gpio(PinTab[i]))
        {
            rtc_gpio_deinit(PinTab[i]);
        }
        conf.pin_bit_mask = 1LL << PinTab[i]; // set all pins to input, no pullup/down/interrupt
        gpio_config(&conf);
    }

    // Route Pins to I2S peripheral using GPIO matrix, last parameter is invert
    gpio_matrix_in(D0,    I2S0I_DATA_IN0_IDX, 0);
    gpio_matrix_in(D1,    I2S0I_DATA_IN1_IDX, 0);
    gpio_matrix_in(D2,    I2S0I_DATA_IN2_IDX, 0);
    gpio_matrix_in(D3,    I2S0I_DATA_IN3_IDX, 0);
    gpio_matrix_in(D4,    I2S0I_DATA_IN4_IDX, 0);
    gpio_matrix_in(D5,    I2S0I_DATA_IN5_IDX, 0);
    gpio_matrix_in(D6,    I2S0I_DATA_IN6_IDX, 0);
    gpio_matrix_in(D7,    I2S0I_DATA_IN7_IDX, 0);
    gpio_matrix_in(VSYNC, I2S0I_V_SYNC_IDX, 0);  // below 3 signals must be high to receive data!
    gpio_matrix_in(0x38,  I2S0I_H_SYNC_IDX, 0);  //0x30 sends 0, 0x38 sends 1
    gpio_matrix_in(HREF,  I2S0I_H_ENABLE_IDX, 0); //+++++++debug!! HREF
    gpio_matrix_in(PCLK,  I2S0I_WS_IN_IDX, 0);

    //PortPins are configured now

    // Power on the I2S peripheral
    periph_module_enable(PERIPH_I2S0_MODULE);

    // Reset I2S and DMA controller
    i2sConfReset(); //??
    // Enable slave receiver mode (WS is input clock/strobe)
    I2S0.conf.rx_slave_mod = 1;
    // Enable parallel mode
    I2S0.conf2.lcd_en = 1;
    // enable camera-mode. Use HSYNC/VSYNC/HREF to control sampling
    I2S0.conf2.camera_en = 1;
    // Configure clock divider
    I2S0.clkm_conf.clkm_div_a = 1; // Fractional clock divider denominator value
    I2S0.clkm_conf.clkm_div_b = 0; // Fractional clock divider numerator value
    I2S0.clkm_conf.clkm_div_num = 2; // Integral I2S clock divider value
    // enable I2S DMA mode
    I2S0.fifo_conf.dscr_en = 1;
    // FIFO configuration
    //two bytes per dword packing
    I2S0.fifo_conf.rx_fifo_mod = 1; //samplemode! 1. no speed difference when using mode 3!
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    //rx channel mode
    I2S0.conf_chan.rx_chan_mod = 1;
    // Clear flags which are used in I2S serial mode
    I2S0.sample_rate_conf.rx_bits_mod = 0;
    I2S0.conf.rx_right_first = 0;
    I2S0.conf.rx_msb_right = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_mono = 0;
    I2S0.conf.rx_short_sync = 0;
    I2S0.timing.val = 0;

    I2S0.int_clr.val = 0x1ffff; // clear all intflags


    // Allocate I2S interrupt
    esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,&isr_i2s, NULL, &isr_i2sHandle);
// install isr_service for pin interrupts
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM); // rising int-prio doesnt improve performance, i tested!
// install vsync int and leave it running
    gpio_isr_handler_add(VSYNC, &isr_vysnc, NULL);
    gpio_set_intr_type(VSYNC, GPIO_INTR_NEGEDGE); //enable int on negedge. GPIO_INTR_DISABLE
// install HREF int and leave it running. optional for debug!
//    gpio_isr_handler_add(HREF, &isr_href, NULL);
//    gpio_set_intr_type(HREF, GPIO_INTR_NEGEDGE); //enable int on negedge. GPIO_INTR_DISABLE
    return init_framebuffer(fbsize);
}


/* initializes the i2s dma buffers and descriptors to standard jpeg.
allocate fixed size framebuffer to hold max size jpeg data

entry:
- the max possible size of the jpeg image (make a good guess!)
exit:
- 0 = Ok
If not OK, data reception is disabled!
- 1 = error
*/
int init_framebuffer(int framesize)
{
    int i,n;
    uint8_t *pbuf;


    I2S_Stop(); // stop possible running capture process and reset I2S/DMA
    memset(DMAdesc, 0, sizeof(DMAdesc)); //preclear complete array of descriptors
    memset(DMAbuf, 0, sizeof(DMAbuf)); // and clear the DMAbuffer too, for debug visibility?!

    pbuf=DMAbuf;

    for (i=0; i<NDESCRIPTORS; i++)
    {
        DMAdesc[i].owner=1;
        DMAdesc[i].size = DMABUFSIZE;
        DMAdesc[i].length = 0;
        DMAdesc[i].buf = pbuf;
        DMAdesc[i].qe.stqe_next=&DMAdesc[(i+1)%NDESCRIPTORS];
        pbuf+= DMABUFSIZE; // we use one big buffer, so only add offset
    }
//    I2S0.rx_eof_num = SampleCount; // set the datasize in DWORDs to be received at which an interrupt will occur. not used
    I2S0.in_link.addr = (uint32_t)&DMAdesc[0]; // set the address of first DMAdesr.
    NumDes = NDESCRIPTORS;
    ESP_LOGI(TAG, "Using Descriptors:%d DMAsize:%d",NumDes,DMABUFSIZE);

    ESP_LOGI(TAG, "+++FrameBuffer Size is:%d",framesize);

// alloc framebuffers only after reset:
    if (!numFB)
    {
        memset(FB, 0, sizeof(FB)); //preclear complete array of FB management data


        n = heap_caps_get_free_size(MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
        ESP_LOGI(TAG, "Detected SPIRAM-Size:%d",n);
        i=n/framesize;
        if (i<2) // we want minimum 2 framebuffers
        {
            ESP_LOGE(TAG, "Framesize:%d Bytes too big for at least 2 Framebuffer! FreeMem:%d Bytes",framesize,n);
            return 1;
        }



        //alloc framebuffers
        i=0;
        for (numFB=0; numFB<NBUFFERS; numFB++)
        {
            if (!i)
            {
                // do DRAM alloc
                n = heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
                n-=80000; //we need space for remaining task stuff
                ESP_LOGI(TAG, "Free DRAM Size:%d",n);
                if (n>framesize)
                {
                    pbuf=heap_caps_malloc(framesize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
                    if (pbuf)
                        ESP_LOGI(TAG, "Allocated one Framebuffer in DRAM, its fast ;)");
                    else
                    {
                        i= 1;
                    }
                }
                else
                    i=1;
            }

            if (i)
            {
                n = heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                ESP_LOGI(TAG, "Free SPIRAM Size:%d",n);
                // do spiram alloc
                pbuf=heap_caps_malloc(framesize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

                if (!pbuf)
                {
                    ESP_LOGE(TAG,"heap_caps_malloc SPIRAM failed!!");
                    numFB=0;
                    return 1;
                }
                ESP_LOGI(TAG, "Allocated Framebuffer in SPIRAM, its sloow ;");
            }


            FB[numFB]->pbuf=pbuf;
        }

        FrameSize=framesize;

        //numFB now contains amount of Framebuffers  available.
    }

    return 0;
}

/* user function.
To get the next available framebuffer from the ringbuffer pool.
This function will run in the context of the streamtask or control task
returns address of framebuffer-struct or NULL=no frame available
*/
struct framebuffer *i2s_getframe(void)
{
    int t;

    if (I2Sstate==0)
    {
        ESP_LOGI(TAG, "init Ringbuffer...");
        //init ringbuffer on changed framesize
        readFB=writeFB=0;  //empty
        FrameBuffer=FB[writeFB]->pbuf;
        vTaskDelay(100 / portTICK_PERIOD_MS); // wait a little for camera to adjust to new framesize
        I2S_Start();
    }
    t=400; //timeout period in ms taskdelay below
    while (t--) // wait for next frame from camera
    {
        if (FrameError)
        {
            ESP_LOGE(TAG, "FrameSize overflow!! Buffer:%d Bytes",FrameError); // your configured framebuffer-size is too small!!
            FrameError=0;
        }

        /* a ringbuffer is empty if readindex==writeindex. it is full if readindex==(writeindex+1)
         the consumer (here) manipulates the readindex.
         the producer manipulates the writeindex.
         if only 2 framebuffers are used, this is effectively a pinp-pong buffer.(see below)
        r w (indexes)
        0 0 = empty
        0 1 = w0 -full, now writes to 1 all the time
        1 1 = r0 -empty, but 1 has already data
        1 0 = w1 -full, now writes to 0 all the time
        0 0 = r1 -empty, but 0 has already data
        ...etc
         */
        if (!(readFB == writeFB)) //if not ringbuffer empty
        {

            return ((struct framebuffer *)&FB[readFB]);

        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // 10ms limits the max framerate to 100 !! 5ms = 200 etc.

        if (I2Sstate==0) // sometimes framesize is changed just in this loop from control task, but streamtask called us.
        {
            ESP_LOGI(TAG, "Restart I2S");
            I2S_Start();
            vTaskDelay(100 / portTICK_PERIOD_MS); // wait a little
        }

    } //endwhile
// return error, we havent found a frame

    return NULL; // nothing found, timeout
}


/*
This user return function is critical.
This function will run in the context of the streamtask or control task
It frees/unlocks the current Framebuffer at readFB-index.
Call it when the last fetched framebuffer is not used any more.
If not called properly, the ringbuffer stalls (thus reading the same framebuffer again in getframe())

If called more than once, it frees all framebuffers currently in queue, thus emptying the ringbuffer.

Each call frees the current read-framebuffer until ringbuffer is empty.
*/
void i2s_unlockfb(void)
{
// advance readindex

//		ESP_LOGI(TAG, "readFB:%d  writeFB:%d",readFB,writeFB); //debug ringbuffer
    if (!(readFB == writeFB)) //if not ringbuffer empty
        readFB=(readFB+1)%numFB; // advance readindex
}

