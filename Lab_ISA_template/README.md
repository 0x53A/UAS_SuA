# Lab ISA Template

STM32G431KB project for the AMR sensor lab assignment, generated with STM32CubeMX.

## Prerequisites

Install [Nix](https://nixos.org/) or use NixOS. All build dependencies are provided via `shell.nix`.

## Build

```sh
nix-shell --run "cmake --preset Debug && cmake --build build/Debug"
```

Or enter the shell interactively:

```sh
nix-shell
cmake --preset Debug
cmake --build build/Debug
```

The output ELF is at `build/Debug/Lab_ISA_template.elf`.

For a release build, replace `Debug` with `Release`.
