.PHONY: build upload calib
build:
	./build.sh blinko_demo
upload:
	./build.sh blinko_demo upload
calib:
	./build.sh strobe_calib upload
