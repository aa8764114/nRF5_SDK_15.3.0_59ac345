#取得git編號
shorthash=$(git log --pretty=format:'%h' -n 1)
echo $shorthash

#創建資料夾_all
mkdir _all

#將app複製到_all
cp ./_build/*.hex ./_all/app.hex

#將bootloader複製到_all
cp /Users/motosawa/Documents/nRF5_SDK_15.3.0_59ac345/examples/dfu/secure_bootloader/pca10040_ble/armgcc/_build/nrf52832_xxaa_s132.hex ./_all/bootloader.hex

#用app在_all資料夾中產生bl-settings
nrfutil settings generate --family NRF52 --application ./_all/app.hex  --application-version 3 --bootloader-version 2 --bl-settings-version 1 ./_all/bl-settings.hex

#把softdevice複製到_all
cp /Users/motosawa/Documents/nRF5_SDK_15.3.0_59ac345/components/softdevice/s132/hex/s132_nrf52_6.1.1_softdevice.hex ./_all/softdevice.hex

#將以上_all內的.hex檔案合併成all.hex
mergehex --merge ./_all/app.hex ./_all/bl-settings.hex ./_all/bootloader.hex ./_all/softdevice.hex --output ./_all/all_$shorthash.hex

#將all.hex複製到桌面
cp ./_all/all_$shorthash.hex /Users/motosawa/Desktop

#清除開發版內的資料
nrfjprog -f nrf52 --eraseall

#刷入all.hex
nrfjprog -f nrf52 --program ./_all/all_$shorthash.hex --sectorerase
nrfjprog -f nrf52 --reset
