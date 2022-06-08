#取得git編號
shorthash=$(git log --pretty=format:'%h' -n 1)
echo $shorthash

#建立_zip資料夾
mkdir _zip

#將private.pem移入_zip
cp ../../.././key/private.pem ./_zip

#複製nrf52840_xxaa.hex到_zip中並改叫update_$shorthash.hex
cp ./_build/*.hex ./_zip/update_$shorthash.hex

#用private.pem和update.hex產生update_$shorthash.zip
nrfutil pkg generate --application ./_zip/update_$shorthash.hex --application-version-string "1.1.0" --hw-version 52 --sd-req 0x00B7 --key-file ./_zip/private.pem ./_zip/update_$shorthash.zip

#將zip複製到桌面
cp ./_zip/update_$shorthash.zip /Users/motosawa/Desktop


