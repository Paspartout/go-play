. ${IDF_PATH}/add_path.sh
export PORT=/dev/ttyUSB0
# For go-emu-launcher gnuboy is locatet at 0x260000, change it to 0x300000 if you wanna use go-play
esptool.py --chip esp32 --port $PORT --baud 921600 write_flash -fs detect --flash_freq 40m --flash_mode qio 0x300000 build/gnuboy-go.bin
