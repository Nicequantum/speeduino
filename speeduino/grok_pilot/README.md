# grok_pilot (Nicequantum/speeduino `grok-pilot` branch)

Marine/AI additions for [Grok Pilot](https://github.com/Nicequantum/ahanu-ecu).

| File | What it is |
|---|---|
| `envelope.h` | **Vendored** from ahanu-ecu `firmware/envelope/envelope.h` (see its provenance header) |
| `proposal.h/.cpp` | Pure logic for the AI proposal command: decode, CRC32, envelope, byte rounding toward old value, RAM log |
| `../grok_pilot_glue.*` | Binds the handler to Speeduino pages (`getPageValue`/`setPageValue`). Teensy 4.1 only |
| `../comms.cpp` `case 'Y'` | The new-protocol command |
| `../../test/grok_pilot_native/` | Desktop GoogleTest suite |

The wire protocol, responses and error codes are specified in ahanu-ecu `docs/PROPOSAL-WIRE-PROTOCOL.md`.

Run the tests:
```bash
cmake -S test/grok_pilot_native -B build/gp && cmake --build build/gp -j && ctest --test-dir build/gp --output-on-failure
```
Build for target: `pio run -e teensy41`. Set the engine with `build_flags = -DGROK_PILOT_ENGINE_ID=1` for starboard.
