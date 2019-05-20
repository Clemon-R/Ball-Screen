/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "mpu.h"

#include "esp_system.h"
#include "tftspi.h"
#include "tft.h"
#include "spiffs_vfs.h"

static const char *TAG = "Main";

#define PIN_NUM_MISO (gpio_num_t)25
#define PIN_NUM_MOSI (gpio_num_t)23
#define PIN_NUM_CLK  (gpio_num_t)19
#define PIN_NUM_CS   (gpio_num_t)22

#define PIN_NUM_DC   (gpio_num_t)21
#define PIN_NUM_RST  (gpio_num_t)18
#define PIN_NUM_BCKL (gpio_num_t)5

#define NBR_FPS (uint8_t)50
#define BALL_SIZE   (uint8_t)32
#define BALL_SPEED  (uint8_t)3

static char tmp_buff[64];

void app_main()
{
    xQueueHandle queue = xQueueCreate(10, sizeof(float *));

    ESP_ERROR_CHECK(mpuInitMaster());
    xTaskCreate(taskMpu, "mpu 6050", 1024 * 2, queue, 10, NULL);

    tft_disp_type = DISP_TYPE_ILI9341;
    _width = 320;  // smaller dimension
    _height = 240; // larger dimension
    max_rdclock = 20000000;

    TFT_PinsInit();

    esp_err_t ret;
    spi_lobo_device_handle_t spi;
    spi_lobo_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=6*320*240
    };
    spi_lobo_device_interface_config_t devcfg={
        .clock_speed_hz=20000000,           //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=-1,                       // we will use external CS pin
		.spics_ext_io_num=PIN_NUM_CS,           // external CS pin
        .flags=LB_SPI_DEVICE_HALFDUPLEX, // ALWAYS SET  to HALF DUPLEX MODE!! for display spi
    };
    ret=spi_lobo_bus_add_device(TFT_HSPI_HOST, &buscfg, &devcfg, &spi);
    assert(ret==ESP_OK);
	disp_spi = spi;

	// ==== Test select/deselect ====
	ret = spi_lobo_device_select(spi, 1);
    assert(ret==ESP_OK);
	TFT_display_init();
    max_rdclock = find_rd_speed();

    // ==== Set SPI clock used for display operations ====
	spi_lobo_set_speed(spi, DEFAULT_SPI_CLOCK);

	font_rotate = 0;
	text_wrap = 0;
	font_transparent = 1;
	font_forceFixed = 0;
	gray_scale = 0;
    TFT_setGammaCurve(DEFAULT_GAMMA_CURVE);
	TFT_setRotation(LANDSCAPE);
	TFT_setFont(DEF_SMALL_FONT, NULL);
    TFT_resetclipwin();

    TFT_fillRect(0,0,80,240,TFT_BLUE);
    TFT_fillRect(80,0,80,240,TFT_GREEN);
    TFT_fillRect(160,0,80,240,TFT_RED);
    TFT_fillRect(240,0,80,240,TFT_WHITE);

    _fg = TFT_WHITE;
    TFT_print("Loading program...", CENTER, CENTER);
	font_transparent = 0;
    float posX = xTaskGetTickCount() % (_width - BALL_SIZE * 2) + BALL_SIZE;
    float posY = xTaskGetTickCount() % (_height - BALL_SIZE * 2) + BALL_SIZE;
    TickType_t  max = pdMS_TO_TICKS(1000 / NBR_FPS);
    TickType_t  time = xTaskGetTickCount() * portTICK_RATE_MS;
    uint8_t counter = 0;
    color_t random_colors[4] = {TFT_GREEN, TFT_WHITE, TFT_RED, TFT_BLUE};
    color_t color = random_colors[xTaskGetTickCount() % 4];
    float   dirX = 0, dirY = 0;
    sprintf(tmp_buff, "%d Fps", counter);
    vTaskDelay(pdMS_TO_TICKS(1000));

    TFT_fillScreen(TFT_BLACK);
    TFT_drawRect(0, 0, _width, _height, TFT_WHITE);
    while (1){
        int32_t  current = xTaskGetTickCount();
        float   *data;

        if (xQueueReceive(queue, &data, 0)){
            dirX = data[0] * BALL_SPEED;
            dirY = data[1] * BALL_SPEED;
            free(data);
        }
        float newPosX = posX + dirX, newPosY = posY + dirY;
        if (newPosX - BALL_SIZE > 0 && newPosX + BALL_SIZE <= _width){
            posX += dirX;
        } else {
            color = random_colors[xTaskGetTickCount() % 4];
        }
        if (newPosY - BALL_SIZE > 0 && newPosY + BALL_SIZE <= _height){
            posY += dirY;
        } else {
            color = random_colors[xTaskGetTickCount() % 4];
        }
        TFT_fillCircle((uint16_t)posX, (uint16_t)posY, BALL_SIZE, color);
        if (current * portTICK_RATE_MS - time >= 1000){
            sprintf(tmp_buff, "%d Fps", counter);
            counter = 0;
            time = current * portTICK_RATE_MS;
        }
        TFT_print(tmp_buff, 0, 0);
        counter++;
        current = max - (xTaskGetTickCount() - current);
        if (max > 1 && current > 0){
            vTaskDelay(current);
        }
        TFT_fillCircle((uint16_t)posX, (uint16_t)posY, BALL_SIZE, TFT_BLACK);
    }
}
