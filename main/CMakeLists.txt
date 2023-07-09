idf_component_register(
        SRCS
            "main.cpp"
            "driver/ac073tc1.cpp" "driver/ac073tc1.hpp"
            "driver/lv_ac073.cpp" "driver/lv_ac073.hpp"

        INCLUDE_DIRS
            "." "driver"

        REQUIRES
            "jianjia_net" "driver" "lvgl"
)
