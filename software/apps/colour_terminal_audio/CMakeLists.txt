add_executable(colour_terminal_audio
	main.c
	tmds_encode_font_2bpp.S
	tmds_encode_font_2bpp.h
)

target_compile_definitions(colour_terminal_audio PRIVATE
	DVI_VERTICAL_REPEAT=2
	)

# We have a lot in SRAM4 (particularly TMDS LUT) but don't need much stack on
# core 1. Probably even 256 bytes would be fine.
target_compile_definitions(colour_terminal_audio PRIVATE
	DVI_DEFAULT_SERIAL_CONFIG=${DVI_DEFAULT_SERIAL_CONFIG}
	PICO_CORE1_STACK_SIZE=0x200
	)

target_link_libraries(colour_terminal_audio
	pico_stdlib
	pico_multicore
	libdvi
)

# pico_enable_stdio_uart(colour_terminal_audio 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(colour_terminal_audio)
