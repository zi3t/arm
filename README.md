# arm

A GLUON six-axis robot arm you drive from the keyboard, in the browser.
The viewer is C11 + raylib compiled to WebAssembly. Kinematics are in
`c/kinematics.c`, and `kinematics.js` is the JavaScript reference.

Archived. It runs as a plain static folder: serve this directory and open `index.html`.

- `make -C c test` checks the C kinematics against `kinematics.fixture.json`
- `node --test kinematics.test.js` checks the JS reference against the same fixture
- `make -C c site` rebuilds `viewer/` (needs Emscripten and raylib built for the web)

Third-party parts (robot model, raylib, Emscripten, Departure Mono) are listed
in `NOTICE.txt`.
