{ pkgs ? import <nixpkgs> {} }:
pkgs.python3.withPackages (ps: with ps; [
  (ps.callPackage ./lightglue.nix)
])
