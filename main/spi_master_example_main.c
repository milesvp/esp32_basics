/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/i2s.h"
#include "soc/i2s_reg.h"


#define I2S_NUM     I2S_NUM_0


#define STEREO_SAMPLES 64

#define PIN_SPI_MISO GPIO_NUM_4
#define PIN_SPI_MOSI GPIO_NUM_2
#define PIN_SPI_CLK  GPIO_NUM_22
#define PIN_SPI_CS   GPIO_NUM_5
#define PIN_SPI_LD   GPIO_NUM_32

#define I2S_MCK_IO      (GPIO_NUM_0)
#define I2S_BCK_IO      (GPIO_NUM_27)
#define I2S_WS_IO       (GPIO_NUM_26)
#define I2S_DO_IO       (GPIO_NUM_23)
#define I2S_DI_IO       (GPIO_NUM_25)


#define LCD_HOST    HSPI_HOST

#define LED_BIT_CNT 8 //number of total SIPO shift registers in bits, rounded up to nearest byte
#define BUT_BIT_CNT 8 //number of total PISO shift registers in bits, rounded up to nearest byte
#define BITS_TRANSFER (LED_BIT_CNT + BUT_BIT_CNT)
#define BYTES_TRANSMIT (BITS_TRANSFER>>3)

#define SPI_BAUD_RATE 8000000


uint8_t TX_buff[BYTES_TRANSMIT] = {0x00};
uint8_t RX_buff[BYTES_TRANSMIT] = {0x00};

QueueHandle_t i2s_queue;

#define DEFAULT_VREF    2400        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   8          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_0;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

#define I2C_MASTER_SCL_IO 19               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 18               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 1 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_LEN 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_LEN 0                           /*!< I2C master doesn't need buffer */

#define BH1750_SENSOR_ADDR 0x23   /*!< slave address for BH1750 sensor */
#define BH1750_CMD_START 0x23   /*!< Operation mode */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

/**
 * @brief test code to operate on BH1750 sensor
 *
 * 1. set operation mode(e.g One time L-resolution mode)
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
static esp_err_t i2c_master_sensor_test(i2c_port_t i2c_num, uint8_t *data_h, uint8_t *data_l)
{
    int ret;
//    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//    i2c_master_start(cmd);
//    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
//    i2c_master_write_byte(cmd, BH1750_CMD_START, ACK_CHECK_EN);
//    i2c_master_stop(cmd);
//    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//    i2c_cmd_link_delete(cmd);
    uint8_t out = BH1750_CMD_START;
    ret = i2c_master_write_to_device(i2c_num, BH1750_SENSOR_ADDR,
                                     &out, 1,
                                     1000 / portTICK_RATE_MS);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(30 / portTICK_RATE_MS);
//    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//    i2c_master_start(cmd);
//    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
//    i2c_master_read_byte(cmd, data_h, ACK_VAL);
//    i2c_master_read_byte(cmd, data_l, NACK_VAL);
//    i2c_master_stop(cmd);
//    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//    i2c_cmd_link_delete(cmd);
    uint8_t in[2] = {0};
    ret = i2c_master_read_from_device(i2c_num, BH1750_SENSOR_ADDR,
                                      in, 2,
                                      1000 / portTICK_RATE_MS);
    *data_h = in[0];
    *data_l = in[1];
    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_LEN, I2C_MASTER_TX_BUF_LEN, 0);
}

static void check_efuse(void)
{
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}


static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

static esp_err_t spi_init(spi_device_handle_t *spi){
  gpio_set_direction(PIN_SPI_LD, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_SPI_LD, 1);
    esp_err_t ret;
    spi_bus_config_t buscfg={
      .mosi_io_num=PIN_SPI_MOSI,
      .miso_io_num=PIN_SPI_MISO,
      .sclk_io_num=PIN_SPI_CLK,
      .quadwp_io_num=-1,
      .quadhd_io_num=-1,
      .data4_io_num=-1,
      .data5_io_num=-1,
      .data6_io_num=-1,
      .data7_io_num=-1,
      .max_transfer_sz=0,
      .flags=0,       ///< Abilities of bus to be checked by the driver. Or-ed value of ``SPICOMMON_BUSFLAG_*`` flags.
      .intr_flags=0
    };
    spi_device_interface_config_t devcfg={
      .command_bits=0,
      .address_bits=0,
      .dummy_bits=0,
      .mode=0,
      .duty_cycle_pos=0,                  ///< Duty cycle of positive clock, in 1/256th increments (128 = 50%/50% duty). Setting this to 0 (=not setting it) is equivalent to setting   this to 128.
      .cs_ena_pretrans=0,                 ///< Amount of SPI bit-cycles the cs should be activated before the transmission (0-16). This only works on half-duplex transactions.
      .cs_ena_posttrans=0,                 ///< Amount of SPI bit-cycles the cs should stay active after the transmission (0-16)
      .clock_speed_hz=SPI_BAUD_RATE,
      .input_delay_ns=0,             /**< Maximum data valid time of slave. The time required between SCLK and MISO
            valid, including the possible clock delay from slave to master. The driver uses this value to give an extra
            delay before the MISO is ready on the line. Leave at 0 unless you know you need a delay. For better timing
            performance at high frequency (over 8MHz), it's suggest to have the right value.
            */
      .spics_io_num=PIN_SPI_CS,               ///< CS GPIO pin for this device, or -1 if not used
      .flags=0,                 ///< Bitwise OR of SPI_DEVICE_* flags
      .queue_size=1,                  //We want to be able to queue 1 transactions at a time
      .pre_cb=NULL,
      .post_cb=NULL
    };

    //Initialize the SPI bus
    ret=spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(LCD_HOST, &devcfg, spi);
    ESP_ERROR_CHECK(ret);
    //Initialize the LCD
    //Initialize the effect displayed

    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        ret = adc1_config_width(width);
        ESP_ERROR_CHECK(ret);
        ret = adc1_config_channel_atten(channel, atten);
        ESP_ERROR_CHECK(ret);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
    return ret;
}

void send_line(spi_device_handle_t spi, uint8_t *TX_buff, uint8_t *RX_buff)
{
    esp_err_t ret;
    //Transaction descriptors. Declared static so they're not allocated on the stack; we need this memory even when this
    //function is finished because the SPI driver needs access to it even while we're already calculating the next line.
    static spi_transaction_t trans;

    //In theory, it's better to initialize trans and data only once and hang on to the initialized
    //variables. We allocate them on the stack, so we need to re-init them each call.
    memset(&trans, 0, sizeof(spi_transaction_t));

    trans.length = BITS_TRANSFER;
    trans.rxlength = BITS_TRANSFER;
    trans.tx_buffer = TX_buff;
    trans.rx_buffer = RX_buff;

    ret=spi_device_polling_transmit(spi, &trans);
    assert(ret==ESP_OK);
}

esp_err_t i2s_init(void){
  esp_err_t ret;

  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX),
      .sample_rate = 48000,
      .bits_per_sample = I2S_BITS_PER_CHAN_32BIT,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
//      .communication_format = I2S_COMM_FORMAT_STAND_MSB,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2, //greater than LEVEL3 causes panic
      .dma_buf_count = 2,
      .dma_buf_len = STEREO_SAMPLES * 2 * 4,
      .use_apll = 1,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 12288000,
      .mclk_multiple = 256,
//      .bits_per_chan = 0
  };

  i2s_pin_config_t pin_config = {
      .mck_io_num = I2S_MCK_IO,
      .bck_io_num = I2S_BCK_IO,
      .ws_io_num = I2S_WS_IO,
      .data_out_num = I2S_DO_IO,
      .data_in_num = I2S_DI_IO                                               //Not used
  };
  ret = i2s_driver_install((i2s_port_t) I2S_NUM, &i2s_config, 4, &i2s_queue);
//  ret = i2s_driver_install((i2s_port_t) I2S_NUM_TX, &i2s_config, 0, NULL);
  ESP_ERROR_CHECK(ret);
  ret = i2s_set_pin(I2S_NUM, &pin_config);
  ESP_ERROR_CHECK(ret);
  return ret;
}

uint32_t TOTAL_READS[40000] = {0};
uint16_t read_index = 0;

void i2sTask(void *param)
{
    i2s_event_t evt;
    esp_err_t ret;
    uint8_t i2sRX[STEREO_SAMPLES * 2 * 4];
    int32_t *samplesRX = (int32_t *)i2sRX;
    uint8_t i2sTX[STEREO_SAMPLES * 2 * 4];
    int32_t *samplesTX = (int32_t *)i2sTX;
    size_t bytesRead = 0;
    size_t bytesWritten = 0;
    volatile int32_t max = 0;
    volatile int32_t min = 0x80000000;
    volatile int i;
    volatile uint32_t tx_val = 0;
    volatile int foo;

    for (i=0; i<STEREO_SAMPLES * 2; i++){
      samplesTX[i] = tx_val++;
    }
    volatile bool starting = true;

    ret = i2s_write(I2S_NUM, i2sTX, STEREO_SAMPLES * 2 * 4, &bytesWritten, portMAX_DELAY);

    while (1)
    {
//        // wait for some data to arrive on the queue
      ret = i2s_read (I2S_NUM, i2sRX, STEREO_SAMPLES * 2 * 4, &bytesRead,    portMAX_DELAY);
      if(starting && tx_val>5000) {
        tx_val = 0;
        starting = false;
      }
      for (i=0; i<STEREO_SAMPLES * 2; i++){
        samplesTX[i] = tx_val++;
        TOTAL_READS[read_index++] = samplesRX[i];
        if (read_index>=40000){
          read_index = 0;
        }
      }
      ret = i2s_write(I2S_NUM, i2sTX, STEREO_SAMPLES * 2 * 4, &bytesWritten, portMAX_DELAY);
      taskYIELD();

//        if (xQueueReceive(i2s_queue, &evt, portMAX_DELAY) == pdPASS)
//        {
//            if (evt.type == I2S_EVENT_RX_DONE)
//            {
//                foo++;
//                bytesRead= 0;
//                max = 0;
//                min = 0x80000000;
//                i2s_read(I2S_NUM, i2sRX, STEREO_SAMPLES * 2, &bytesRead, 10);
////                printf("queue space avail: %d\n", uxQueueSpacesAvailable(i2s_queue));
//                if (uxQueueSpacesAvailable(i2s_queue) < 2){
//                  printf("DANGER QUEUE MAX\n");
//                }
//                printf("ret: %d\n", ret);
//                for (int x=0; x<STEREO_SAMPLES * 2; x++){
//                  printf("%d: %d\n", samplesTX[x], samplesRX[x]);
//                }
//                for (int i=0;i<STEREO_SAMPLES * 2;i++){
//                  max = (samplesRX[i]>max)?samplesRX[i]:max;
//                  min = (samplesRX[i]<min)?samplesRX[i]:min;
//                }
//                if (max > 0){
//                  printf("I2S max: %d, min: %d\n", max, min);
//                }
////                do
////                {
////                    foo++;
//////                    // read data from the I2S peripheral
//////                    // read from i2s
////                    i2s_read(I2S_NUM_0, i2sRX, STEREO_SAMPLES * 2, &bytesRead, 10);
//////                    // process the raw data
//////                    //sampler->processI2SData(i2sRX, bytesRead);
////                } while (bytesRead > 0);
//            }
//        }
    }
}

void app_main(void)
{
  printf("starting spi\n");
  esp_err_t ret;

  spi_device_handle_t spi;
  ESP_ERROR_CHECK(spi_init(&spi));

  ESP_ERROR_CHECK(i2c_master_init());
  uint8_t sensor_data_h, sensor_data_l;

  ESP_ERROR_CHECK(i2s_init());

  xTaskCreatePinnedToCore(i2sTask,
                          "i2s Task",
                          4096,
                          NULL,
                          3,
                          NULL, //&readerTaskHandle,
                          1);


  volatile unsigned int foo = 0;
  TX_buff[0] = 0xFF;
  TX_buff[1] = 0xFF;
  send_line(spi, TX_buff, RX_buff);
  uint32_t adc_reading = 0;
  while (1){

    foo += 1;

    if (~(RX_buff[0]) & 0x8){
      TX_buff[0] = 0x1;
      TX_buff[1] = 0x1;
    } else {
      TX_buff[0] = 0x00;
      TX_buff[1] = 0x00;
    }
//    	TX_buff[2] = 0x33;
    gpio_set_level(PIN_SPI_LD, 0);
    __asm__ __volatile__("nop;nop;"); //hold LD low at least 4ns
    gpio_set_level(PIN_SPI_LD, 1);

    send_line(spi, TX_buff, RX_buff);
    vTaskDelay(10 / portTICK_PERIOD_MS);


//        last_reading = adc_reading;
    adc_reading = 0;
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
      if (unit == ADC_UNIT_1) {
        adc_reading += adc1_get_raw((adc1_channel_t)channel);
      } else {
        int raw;
        adc2_get_raw((adc2_channel_t)channel, width, &raw);
        adc_reading += raw;
      }
    }
    adc_reading /= NO_OF_SAMPLES;
    //Convert adc_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

    if (foo % 100 == 0){
      ret = i2c_master_sensor_test(I2C_MASTER_NUM, &sensor_data_h, &sensor_data_l);
      if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGE("i2c-example", "I2C Timeout");
      } else if (ret == ESP_OK) {
        printf("*******************\n");
        printf("MASTER READ SENSOR( BH1750 )\n");
        printf("*******************\n");
        printf("data_h: %02x\n", sensor_data_h);
        printf("data_l: %02x\n", sensor_data_l);
        printf("sensor val: %.02f [Lux]\n", (sensor_data_h << 8 | sensor_data_l) / 1.2);
      } else {
          ESP_LOGW("i2c-example", "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
      }
      printf("%x %x,  Raw: %d\tVoltage: %dmV\n", RX_buff[0], RX_buff[1], adc_reading, voltage);
    }


  }
}
