zephyr_library()

zephyr_library_sources_ifdef(CONFIG_PAW3212 src/paw3212.c)
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
