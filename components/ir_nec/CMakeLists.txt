idf_component_register(SRCS "ir_nec_encoder.c" "ir_nec_parser.c" "ir_nec.c"
                       PRIV_REQUIRES driver events common esp_ringbuf
                       INCLUDE_DIRS "include")