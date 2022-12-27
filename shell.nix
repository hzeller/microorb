# This is a nix-shell for use with the nix package manager.
# If you have nix installed, you may simply run `nix-shell`
# in this repo, and have all dependencies ready in the new shell.

{ pkgs ? import <nixpkgs> {} }:
pkgs.mkShell {
  buildInputs = with pkgs;
    [
      gnumake

      # Firmware
      pkgsCross.avr.buildPackages.gcc8
      avrdude
      python

      # Host software
      libmicrohttpd
      libusb-compat-0_1
      ncurses

      clang-tools
    ];
}
