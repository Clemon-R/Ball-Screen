#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "mpu.h"

static const char *TAG = "MPU-6050";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO CONFIG_I2C_SLAVE_SCL               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO CONFIG_I2C_SLAVE_SDA               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM I2C_NUMBER(CONFIG_I2C_SLAVE_PORT_NUM) /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define BH1750_SENSOR_ADDR CONFIG_BH1750_ADDR   /*!< slave address for BH1750 sensor */
#define BH1750_CMD_START CONFIG_BH1750_OPMODE   /*!< Operation mode */
#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define REGISTER_WHOIAM 0x75
#define REGISTER_PVM1 0x6B
#define REGISTER_TMP_H 0x41
#define REGISTER_TMP_L 0x42
#define REGISTER_GYROX_H 0x43
#define REGISTER_GYROX_L 0x44
#define REGISTER_GYROY_H 0x45
#define REGISTER_GYROY_L 0x46
#define REGISTER_GYROZ_H 0x47
#define REGISTER_GYROZ_L 0x48

#define REGISTER_ACCEX_H 0x3b
#define REGISTER_ACCEX_L 0x3c
#define REGISTER_ACCEY_H 0x3d
#define REGISTER_ACCEY_L 0x3e
#define REGISTER_ACCEZ_H 0x3f
#define REGISTER_ACCEZ_L 0x40

#define RGB_R (gpio_num_t)0

SemaphoreHandle_t print_mux = NULL;



static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}
/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t readBytes(i2c_port_t i2c_num, uint8_t *data, size_t size)
{
    if (size == 0 || data == NULL) {
        return ESP_FAIL;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 0);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t readByte(i2c_port_t i2c_num, uint8_t *data)
{
    if (data == NULL) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 0);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t writeByte(i2c_port_t i2c_num, uint8_t registerId, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, registerId, ACK_CHECK_EN);
    if (data != NULL){
        i2c_master_write_byte(cmd, *data, ACK_CHECK_EN);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 0);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t writeBytes(i2c_port_t i2c_num, uint8_t registerId, uint8_t *data, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, registerId, ACK_CHECK_EN);
    if (size != 0 && data != NULL){
        i2c_master_write(cmd, data, size, ACK_CHECK_EN);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 0);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief test code to operate on BY-521 sensor
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

static esp_err_t whomIAm(i2c_port_t i2c_num)
{
    uint8_t data;
    int ret;

    ESP_LOGI(TAG, "Trying 'WHO I AM' request...");
    ret = writeByte(i2c_num, REGISTER_WHOIAM, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Request KO");
        return ret;
    }
    ret = readByte(i2c_num, &data);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Request KO");
        return ret;
    }
    return data != ESP_SLAVE_ADDR ? ESP_FAIL : ESP_OK;
}

static esp_err_t powerManagement1(i2c_port_t i2c_num)
{
    uint8_t data = 0x00;
    int ret;

    ESP_LOGI(TAG, "Trying 'POWER MANAGEMENT 1' request...");
    ret = writeByte(i2c_num, REGISTER_PVM1, &data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Request KO");
        return ret;
    }
    data = 0x00;
    ret = readByte(i2c_num, &data);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Request KO");
        return ret;
    }
    disp_buf(&data, 1);
    return ret;
}

static esp_err_t sendRequest(i2c_port_t i2c_num, uint8_t registerId, uint8_t *data)
{
    int ret;
    
    ret = writeByte(i2c_num, registerId, data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Request KO: %d", ret);
        return ret;
    }
    if (data == NULL){
        return ret;
    }
    ret = readByte(i2c_num, data);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Request KO: %d", ret);
        return ret;
    }
    return ret;
}

static esp_err_t sendRequests(i2c_port_t i2c_num, uint8_t registerId, uint8_t *data, size_t size)
{
    int ret;
    
    ret = writeByte(i2c_num, registerId, data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Request KO: %d", ret);
        return ret;
    }
    if (data == NULL){
        return ret;
    }
    ret = readBytes(i2c_num, data, size);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Request KO: %d", ret);
        return ret;
    }
    return ret;
}

static void taskRGB(void *arg)
{
    gpio_pad_select_gpio(RGB_R);
    gpio_set_direction(RGB_R, GPIO_MODE_OUTPUT);
    while (1) {
        gpio_set_level(RGB_R, 1);
        vTaskDelay(200 / portTICK_RATE_MS);
        gpio_set_level(RGB_R, 0);
        vTaskDelay(200 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

/**
 * @brief i2c master initialization
 */
esp_err_t mpuInitMaster()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief test function to show buffer
 */

void taskMpu(void *arg)
{
    int ret;
    float    angleX = 0, angleY = 0, angleZ = 0;
    int cnt = 0;
    TaskHandle_t handler = NULL;
    xQueueHandle    queue = arg != NULL ? (xQueueHandle)arg : NULL;

    printf("*******************\n");
    printf("TASK SENSOR( GY-521 )\n");
    printf("*******************\n");
    ret = whomIAm(I2C_MASTER_NUM);
    if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "I2C Timeout");
    } else if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Sensor perfectly working !");
    } else {
        ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(powerManagement1(I2C_MASTER_NUM));
    ret = writeByte(I2C_MASTER_NUM, 0x1c, (uint8_t *)0);
    ESP_ERROR_CHECK(ret);
    union{
        struct 
        {
        int16_t accex; 
        int16_t accey;
        int16_t accez;
        } axes;
        uint8_t buff[6];
    } data;
    while (1) {
        //int16_t tmp, accex, accey, accez;
        //uint8_t *buffer;

        //ESP_LOGI(TAG, "TASK Turn: %d", cnt++);

        //ESP_LOGI(TAG, "TASK[%d] Temperature Measurement", task_idx);
        /*buffer = (uint8_t *)&tmp;
        ret = sendRequest(I2C_MASTER_NUM, REGISTER_TMP_H, buffer);
        if (ret != ESP_OK){
            continue;
        }
        ret = sendRequest(I2C_MASTER_NUM, REGISTER_TMP_L, buffer + 1);
        if (ret != ESP_OK){
            continue;
        }
        float result = ((float)tmp)/340.0f+36.53F;
        //ESP_LOGI(TAG, "Final value: %.2f°C", result);
        if (result > 40 && handler == NULL){
            xTaskCreate(taskRGB, "RGB RED", 1024, NULL, tskIDLE_PRIORITY, &handler);
        } else if (handler != NULL) {
            vTaskDelete(handler);
            handler = NULL;
        }*/
        //ESP_LOGI(TAG, "------------------------TASK Accelero Measurement");
        ret = sendRequests(I2C_MASTER_NUM, REGISTER_ACCEX_H, data.buff, 6);
        if (ret != ESP_OK){
            continue;
        }
        angleX = data.axes.accex / 16384.0f;
        angleY = data.axes.accey / 16384.0f;
        angleZ = data.axes.accez / 16384.0f;
        if (queue && !xQueueIsQueueFullFromISR(queue)){
            //ESP_LOGI(TAG, "Final value - X:%i, Y:%i, Z:%i", data.axes.accex, data.axes.accey, data.axes.accez);
            ESP_LOGI(TAG, "Final value - X:%.2f, Y:%.2f, Z:%.2f", angleX, angleY, angleZ);
            float   *data = (float *)malloc(sizeof(float) * 2);
            data[0] = angleX;
            data[1] = angleY;
            xQueueSendFromISR(queue, &data, NULL);
        }
    }
    vTaskDelete(NULL);
}