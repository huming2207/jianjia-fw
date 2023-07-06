idf_component_register(
        SRCS
            "main.cpp"
            "driver/ac073tc1.cpp" "driver/ac073tc1.hpp"

        INCLUDE_DIRS
            "."

        REQUIRES
            "jianjia_net" "driver"
)
