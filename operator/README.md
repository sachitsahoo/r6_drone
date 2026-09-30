# operator — laptop ground station

## What it does

Reads the Xbox controller (SDL2), displays video with a telemetry overlay, logs the session,
replays logs, tunes parameters over the protocol, and hosts the simulator front end.

## How it fits the architecture

The human interface, and the only tier where a person is in the loop. It sends commands at
~50 Hz and receives telemetry at 50-100 Hz. It is deliberately not trusted for safety: the
MCU's watchdog and safety state machine assume the operator link can vanish mid-command.

## Key design decisions

- **Degrees and human units in the UI only.** Everything on the wire and in the firmware is
  SI with unit suffixes. Conversion happens at the display boundary, in one place.
- **Logs are raw protocol frames**, not decoded records. Replay then reuses the same decoder
  as live operation, so a decoder bug cannot hide behind a second parsing path — and a log
  recorded before a field was understood is still readable afterward.
- **Arming is an explicit operator action**, and leaving `FAULT` or `ESTOP` requires another.
  No automatic recovery, no arming on connect.

## Known limitations

- Empty as of Phase 1.
- Parameter tuning depends on the protocol's get/set messages, which are not designed yet.
- Autonomy is explicitly out of scope; no hooks for it exist or should be added.

## How to test

```
pytest
```

Controller input and rendering need a human; protocol handling, logging, and replay do not,
and those are what the tests cover.
