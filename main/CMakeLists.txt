idf_component_register(SRCS "bme680.c" "main.c" "i2c.c" "sps30.c" "ssd1306.c"
                    INCLUDE_DIRS "")

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
