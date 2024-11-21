idf_component_register( SRCS "manager.c" "gpio.c" "spi_xfer.c" "gpio_decoder.c" "HA_pulse_sync.c"
                             "mmc5983ma.c" "i2c_xfer.c" "bt_spp.c" "ms5611.c" "bmi088.c"
                             "calibration.c" "led.c" "power.c" "packet_loss.c" "battery.c" 
                             "ahrs_env.c" "uart.c" "resend_packet_method.c" "connection_mode.c" "prisonator_external_flash.c"
                        INCLUDE_DIRS "."
                        REQUIRES AHRS bt nvs_flash app_update esp_adc_cal
                        )

#  #
#  # ULP support additions to component CMakeLists.txt.
#  #
#  # 1. The ULP app name must be unique (if multiple components use ULP).
#  set(ulp_app_name ulp_${COMPONENT_NAME})
#  #
#  # 2. Specify all assembly source files.
#  #    Files should be placed into a separate directory (in this case, ulp/),
#  #    which should not be added to COMPONENT_SRCS.
#  set(ulp_s_sources "ulp/pulse_cnt.S")
#  #
#  # 3. List all the component source files which include automatically
#  #    generated ULP export file, ${ulp_app_name}.h:
#  set(ulp_exp_dep_srcs "ulp_sleep.c")
#  #
#  # 4. Call function to build ULP binary and embed in project using the argument
#  #    values above.
#  ulp_embed_binary(${ulp_app_name} "${ulp_s_sources}" "${ulp_exp_dep_srcs}")
