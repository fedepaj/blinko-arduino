.PHONY: build upload calib
build:
	./build.sh BlinkoDemo
upload:
	./build.sh BlinkoDemo upload
calib:
	./build.sh StrobeCalibration upload
