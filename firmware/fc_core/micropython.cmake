# モジュール用のライブラリターゲットを作成
add_library(usermod_fc_core INTERFACE)

# コンパイルするC言語ソースファイルを指定
target_sources(usermod_fc_core INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/fc_core.c
)

# インクルードパス（このディレクトリ内のヘッダー等を探せるようにする）
target_include_directories(usermod_fc_core INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

# MicroPythonのビルド本体 (usermod) にリンクする
target_link_libraries(usermod INTERFACE usermod_fc_core)
