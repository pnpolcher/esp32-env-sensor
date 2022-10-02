idf_component_register(SRCS "utils.c" "bme68x_wrapper.c" "bme68x.c" "bme68x.c" "main.c" "i2c.c" "sps30.c" "ssd1306.c"
                    INCLUDE_DIRS "")

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
