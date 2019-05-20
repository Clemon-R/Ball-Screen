deps_config := \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/app_trace/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/aws_iot/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/bt/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/driver/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/esp32/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/esp_adc_cal/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/esp_event/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/esp_http_client/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/esp_http_server/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/ethernet/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/fatfs/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/freemodbus/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/freertos/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/heap/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/libsodium/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/log/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/lwip/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/mbedtls/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/mdns/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/mqtt/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/nvs_flash/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/openssl/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/pthread/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/spi_flash/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/tcpip_adapter/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/vfs/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/wear_levelling/Kconfig \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/bootloader/Kconfig.projbuild \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/esptool_py/Kconfig.projbuild \
	/home/raphael/rtone/sensor/main/Kconfig.projbuild \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/components/partition_table/Kconfig.projbuild \
	/home/raphael/espressif/tools/esp32-sdk/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)

ifneq "$(IDF_CMAKE)" "n"
include/config/auto.conf: FORCE
endif

$(deps_config): ;
