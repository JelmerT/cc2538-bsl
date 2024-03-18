{
  description = "Script to upload firmware via the serial boot loader onto the CC13xx, CC2538 and CC26xx SoC.";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs = inputs@{ flake-parts, ... }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      debug = true;
      imports = [];
      systems = [
        "x86_64-linux"
        "aarch64-linux"
      ];
      perSystem = { self', config, pkgs, system, ... }: {
        packages.default =
          let
            python = pkgs.python3.withPackages (p: [
              p.intelhex
              p.pyserial
              p.python-magic
            ]);
            cc2538-bsl_py = builtins.path {
              name = "cc2538-bsl.py";
              path = ./cc2538-bsl.py;
            };
          in
          pkgs.writeShellScriptBin "cc2538-bsl.sh" ''
            ${python}/bin/python3 ${cc2538-bsl_py} "$@"
          '';
      };
    };
}
