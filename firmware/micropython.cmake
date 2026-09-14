# firmware/micropython.cmake

# st7789モジュールをインクルード
include(${CMAKE_CURRENT_LIST_DIR}/st7789_mpy/micropython.cmake)

# fc_coreモジュールをインクルード
include(${CMAKE_CURRENT_LIST_DIR}/fc_core/micropython.cmake)
