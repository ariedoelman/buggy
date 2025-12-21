# Buggy

## Flashing via the Raspberry Pi Debug Probe

 - The `.cargo/config.toml` runner now uses `probe-rs run --chip RP2040 --protocol swd --speed 4000`, so `cargo run --target thumbv6m-none-eabi` flashes the board directly through the probe.
 - Use `scripts/flash-with-probe.sh` when you only want to program the chip without starting a debugger; it calls `probe-rs flash` and reuses the same ELF that cargo built.

## Debugging

 - Launch the `rp2040-project` configuration in `.vscode/launch.json`; it starts `probe-rs dap-server`, flashes the ELF, and opens RTT/defmt output via the probe.
 - The `Embed.toml` debug sections enable GDB plus RTT collection over `probe-rs`, matching the probe settings used in the VS Code launch config.

## UART pins 1 & 2 (GP0/GP1)

 - Pins 1 and 2 on the Pico/W header expose `UART0` TX (GP0) and RX (GP1). These match the user-visible UART on the board and are now free for your UART console.
- Hook a 3.3 V TTL UART adapter (TX/RX crossed, plus GND) to those pins when the board is running; UART0 can be initialized in software to communicate with your host.
 - Always share the Pico/W ground with your adapter and avoid 5 V signals on GP0/GP1 to protect the RP2040.
