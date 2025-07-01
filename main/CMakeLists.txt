set(SOURCES
        "protocols/protocol.cc"
        "system_info.cc"
        "settings.cc"
        "background_task.cc"
        "wifi_manager.cc"
        "myApp.cc"
        "stm32_comm.cc"
        "board/board.cc"
        "board/wifi_board.cc"
        "board/compact_wifi_board_lcd.cc"
        "board/button.cc"
        "audio_codecs/audio_codec.cc"
        "audio_codecs/no_audio_codec.cc"
        "utils/base64_encoder.cc"
        "http/http_server.cc"
        "http/ai_role.cc"
        "http/http_client.cc"
        "stm32_comm.cc"
        "main.cc"
        )
set(INCLUDE_DIRS "." "protocols" "board" "utils" "http")


list(APPEND SOURCES "protocols/websocket_protocol.cc")
idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS ${INCLUDE_DIRS}
                    EMBED_TXTFILES "../assets/http_server.html"  # 文本格式
                    EMBED_FILES "../assets/role_list.json" 
                    WHOLE_ARCHIVE
                    )


