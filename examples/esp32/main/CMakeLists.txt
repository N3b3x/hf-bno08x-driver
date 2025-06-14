idf_component_register(
    SRCS "main.cpp" \
         "../../src/BNO085.cpp" \
         "../../src/dfu/dfu_bno.cpp" \
         "../../src/dfu/dfu_fsp200.cpp" \
         "../../src/rvc/rvc.cpp" \
         "../../src/dfu/firmware-bno.c" \
         "../../src/dfu/firmware-fsp.c" \
         "../../src/sh2/euler.c" \
         "../../src/sh2/sh2.c" \
         "../../src/sh2/sh2_SensorValue.c" \
         "../../src/sh2/sh2_util.c" \
         "../../src/sh2/shtp.c" 
    INCLUDE_DIRS "../../src" "../../src/dfu" "../../src/sh2" "../../src/rvc" "../common"
)
