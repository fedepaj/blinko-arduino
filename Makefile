.PHONY: build upload calib
build:
	./build.sh rslog_demo
upload:
	./build.sh rslog_demo upload
calib:
	./build.sh strobe_calib upload
