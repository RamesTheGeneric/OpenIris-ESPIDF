idf_component_register(SRCS "UDPStream/UDPStream.cpp"
  INCLUDE_DIRS "UDPStream"
  REQUIRES ReedSolomon StreamServer esp32-camera lwip esp_wifi
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=format=)
