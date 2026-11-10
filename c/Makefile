# C port of ../kinematics.js. `make test` checks it against the same fixture
# and behaviour as ../kinematics.test.js. `make viewer` builds the raylib arm
# viewer for this machine (Homebrew raylib); `make site` builds the /arm/
# module with Emscripten against a raylib compiled for the web (RAYLIB_WEB).
CC ?= cc
CFLAGS ?= -std=c11 -O2 -Wall -Wextra -Wpedantic -Werror
BUILD := build

MESHES := ../meshes
FONT_WOFF2 := ../fonts/DepartureMono-Regular.woff2
RAYLIB_WEB ?= $(HOME)/Developer/Personal/raylib/src

.PHONY: test wasm viewer site clean

test: $(BUILD)/kinematics_test
	./$(BUILD)/kinematics_test

$(BUILD)/fixture.h $(BUILD)/gluon.h &: gen-fixture.mjs ../kinematics.fixture.json ../gluon.urdf
	node gen-fixture.mjs

$(BUILD)/kinematics_test: kinematics_test.c kinematics.c kinematics.h $(BUILD)/fixture.h
	$(CC) $(CFLAGS) kinematics_test.c kinematics.c -lm -o $@

wasm: $(BUILD)/kinematics.o.wasm

$(BUILD)/kinematics.o.wasm: kinematics.c kinematics.h
	@mkdir -p $(BUILD)
	emcc -std=c11 -O2 -Wall -Wextra -Werror -c kinematics.c -o $@

# raylib reads TrueType, not WOFF2; convert the site's Departure Mono once.
$(BUILD)/DepartureMono.ttf: $(FONT_WOFF2)
	@mkdir -p $(BUILD)
	uv run --quiet --with fonttools --with brotli python -c \
		"import sys; from fontTools.ttLib import TTFont; f = TTFont(sys.argv[1]); f.flavor = None; f.save(sys.argv[2])" $< $@

viewer: $(BUILD)/viewer

$(BUILD)/viewer: viewer.c kinematics.c kinematics.h $(BUILD)/gluon.h $(BUILD)/DepartureMono.ttf
	$(CC) -std=c11 -O2 -Wall -Wextra -DMESH_DIR='"$(MESHES)"' -DFONT_FILE='"$(BUILD)/DepartureMono.ttf"' viewer.c kinematics.c \
		$$(pkg-config --cflags --libs raylib) -lm -o $@

# The /arm/ module, committed to ../viewer/ because CI has no
# Emscripten. -O3 inlines more of the C, so fewer function boundaries survive;
# only the _web_* exports and FS/HEAPF64 keep readable names. Meshes load
# from ../meshes at runtime; the font ships beside it.
SITE := ../viewer

site: $(SITE)/arm.mjs $(SITE)/DepartureMono.ttf

$(SITE)/arm.mjs: viewer.c kinematics.c kinematics.h $(BUILD)/gluon.h $(RAYLIB_WEB)/libraylib.web.a
	@mkdir -p $(SITE)
	emcc -std=c11 -O3 -Wall -Wextra -DPLATFORM_WEB -I$(RAYLIB_WEB) viewer.c kinematics.c $(RAYLIB_WEB)/libraylib.web.a \
		-sUSE_GLFW=3 -sALLOW_MEMORY_GROWTH -sMODULARIZE -sEXPORT_ES6 -sEXPORT_NAME=createArmViewer \
		-sENVIRONMENT=web -sFORCE_FILESYSTEM -sEXPORTED_RUNTIME_METHODS=FS,HEAPF64 -o $@

$(SITE)/DepartureMono.ttf: $(BUILD)/DepartureMono.ttf
	@mkdir -p $(SITE)
	cp $< $@

clean:
	rm -rf $(BUILD)
