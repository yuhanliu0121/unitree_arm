# Contributing

Keep vendor-independent protocol logic separate from Unitree DDS and serial
I/O. Normal motion must remain a complete seven-joint snapshot, and DDS callback
threads must never access the servo bus directly.

Before submitting a change:

1. build the host target with warnings enabled;
2. run `ctest --output-on-failure`;
3. build the onboard target on the D1 computer without installing it;
4. run the consuming project's simulation regression;
5. document any change to packet layout, topics or deployment procedure.

Physical-arm tests must state their maximum commanded joint displacement and
must not silently install or restart the onboard service.
